#include "game.hpp"
#include "platform.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace {

using Mat4 = std::array<float, 16>;  // column-major: index = i44(row, col)

template <size_t N>
Vec<N> add(Vec<N> a, const Vec<N>& b) {
    for (size_t i = 0; i < N; ++i)
        a[i] += b[i];
    return a;
}

constexpr int i44(int row, int col) {
    return col * 4 + row;
}

Mat4 perspective(float fov_y, float aspect, float z_near, float z_far) {
    Mat4 m{};
    const float f = 1.0f / std::tan(fov_y / 2.0f);
    m[i44(0, 0)] = f / aspect;
    m[i44(1, 1)] = f;
    m[i44(2, 2)] = (z_far + z_near) / (z_near - z_far);
    m[i44(3, 2)] = -1.0f;
    m[i44(2, 3)] = (2.0f * z_far * z_near) / (z_near - z_far);
    return m;
}

Mat4 scale(float x, float y, float z) {
    Mat4 m{};
    m[i44(0, 0)] = x;
    m[i44(1, 1)] = y;
    m[i44(2, 2)] = z;
    m[i44(3, 3)] = 1.0f;
    return m;
}

Mat4 quat_translation(Quat q, Vec<3> t) {
    float x = q[0], y = q[1], z = q[2], w = q[3];
    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;
    Mat4 m{};
    m[i44(0, 0)] = 1.0f - 2.0f * (yy + zz);
    m[i44(1, 0)] = 2.0f * (xy + wz);
    m[i44(2, 0)] = 2.0f * (xz - wy);
    m[i44(0, 1)] = 2.0f * (xy - wz);
    m[i44(1, 1)] = 1.0f - 2.0f * (xx + zz);
    m[i44(2, 1)] = 2.0f * (yz + wx);
    m[i44(0, 2)] = 2.0f * (xz + wy);
    m[i44(1, 2)] = 2.0f * (yz - wx);
    m[i44(2, 2)] = 1.0f - 2.0f * (xx + yy);
    m[i44(0, 3)] = t[0];
    m[i44(1, 3)] = t[1];
    m[i44(2, 3)] = t[2];
    m[i44(3, 3)] = 1.0f;
    return m;
}

Vec<3> rotate(Quat q, Vec<3> v) {
    float x = q[0], y = q[1], z = q[2], w = q[3];
    return {
        (1.0f - 2.0f * (y * y + z * z)) * v[0] + 2.0f * (x * y - w * z) * v[1] +
            2.0f * (x * z + w * y) * v[2],
        2.0f * (x * y + w * z) * v[0] + (1.0f - 2.0f * (x * x + z * z)) * v[1] +
            2.0f * (y * z - w * x) * v[2],
        2.0f * (x * z - w * y) * v[0] + 2.0f * (y * z + w * x) * v[1] +
            (1.0f - 2.0f * (x * x + y * y)) * v[2],
    };
}

// Inverse of a rigid [R|t] transform: [R^T | -R^T t].
Mat4 inverse_rigid(const Mat4& m) {
    Mat4 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r[i44(i, j)] = m[i44(j, i)];
    for (int i = 0; i < 3; ++i) {
        float s = 0.0f;
        for (int j = 0; j < 3; ++j)
            s += r[i44(i, j)] * m[i44(j, 3)];
        r[i44(i, 3)] = -s;
    }
    r[i44(3, 3)] = 1.0f;
    return r;
}

Mat4 mul(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int c = 0; c < 4; ++c)
        for (int row = 0; row < 4; ++row) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k)
                s += a[i44(row, k)] * b[i44(k, c)];
            r[i44(row, c)] = s;
        }
    return r;
}

GLuint compile_shader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(s, 512, nullptr, log);
        glDeleteShader(s);
        throw std::runtime_error(std::string("shader compile: ") + log);
    }
    return s;
}

GLuint link_program(GLuint vs, GLuint fs) {
    GLuint p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);
    GLint ok;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(p, 512, nullptr, log);
        glDeleteProgram(p);
        throw std::runtime_error(std::string("program link: ") + log);
    }
    return p;
}

// Scene constants.
constexpr float DRONE_Z_SCALE = 0.15f;  // model Z-thickness for drones
constexpr float FLOOR_HALF = 20.0f;     // floor spans +/-FLOOR_HALF on X/Y
constexpr float GRID_STEP = 1.0f;
constexpr float GRID_Z = 0.001f;  // slightly above floor to avoid z-fighting
// constexpr Vec<3> CAM_OFFSET_VEC{0.5f, 0.0f, 0.0f};  // front (cockpit)
constexpr Vec<3> CAM_OFFSET_VEC{-1.5f, 0.0f, 0.5f};  // chase (behind, slightly up)

// Fixed rotation from drone axes to OpenGL camera axes. Cameras look down
// local -z (forward) with +y up; we want forward = drone +x, up = drone +z,
// right = drone -y. Equivalently a 120-degree rotation about (1,-1,-1)/sqrt3.
// Stored column-major: col0 = right = -drone_y, col1 = up = drone_z,
// col2 = backward (+z_cam) = -drone_x.
constexpr Mat4 CAM_ROTATION = {
    0.0f,  -1.0f, 0.0f, 0.0f,  // col0: right  = drone -y
    0.0f,  0.0f,  1.0f, 0.0f,  // col1: up     = drone +z
    -1.0f, 0.0f,  0.0f, 0.0f,  // col2: back   = drone -x
    0.0f,  0.0f,  0.0f, 1.0f,
};

constexpr float CUBE[] = {
    -0.5f, -0.5f, -0.5f, -0.5f, 0.5f,  -0.5f, -0.5f, 0.5f,  0.5f,  -0.5f, -0.5f, -0.5f,
    -0.5f, 0.5f,  0.5f,  -0.5f, -0.5f, 0.5f,  0.5f,  -0.5f, -0.5f, 0.5f,  0.5f,  0.5f,
    0.5f,  0.5f,  -0.5f, 0.5f,  -0.5f, -0.5f, 0.5f,  -0.5f, 0.5f,  0.5f,  0.5f,  0.5f,
    -0.5f, -0.5f, -0.5f, 0.5f,  -0.5f, 0.5f,  0.5f,  -0.5f, -0.5f, -0.5f, -0.5f, -0.5f,
    -0.5f, -0.5f, 0.5f,  0.5f,  -0.5f, 0.5f,  -0.5f, 0.5f,  -0.5f, 0.5f,  0.5f,  -0.5f,
    0.5f,  0.5f,  0.5f,  -0.5f, 0.5f,  -0.5f, 0.5f,  0.5f,  0.5f,  -0.5f, 0.5f,  0.5f,
    -0.5f, -0.5f, -0.5f, 0.5f,  -0.5f, -0.5f, 0.5f,  0.5f,  -0.5f, -0.5f, -0.5f, -0.5f,
    0.5f,  0.5f,  -0.5f, -0.5f, 0.5f,  -0.5f, -0.5f, -0.5f, 0.5f,  0.5f,  0.5f,  0.5f,
    0.5f,  -0.5f, 0.5f,  -0.5f, -0.5f, 0.5f,  -0.5f, 0.5f,  0.5f,  0.5f,  0.5f,  0.5f,
};

}  // namespace

struct Renderer::State {
    GLuint program = 0;
    GLuint drone_vao = 0;
    GLuint drone_vbo = 0;

    GLuint floor_vao = 0;
    GLuint floor_vbo = 0;
    GLuint grid_vao = 0;
    GLuint grid_vbo = 0;
    GLsizei grid_vertices = 0;

    GLint u_mvp = -1;
    GLint u_color = -1;
    Mat4 projection{};
};

Renderer::Renderer() : state(std::make_unique<State>()) {
    const char* v_src =
        "#version 330 core\n"
        "layout(location=0) in vec3 pos;\n"
        "uniform mat4 u_mvp;\n"
        "void main(){ gl_Position = u_mvp * vec4(pos, 1.0); }\n";
    const char* f_src =
        "#version 330 core\n"
        "out vec4 frag;\n"
        "uniform vec3 u_color;\n"
        "void main(){ frag = vec4(u_color, 1.0); }\n";

    GLuint vs = compile_shader(GL_VERTEX_SHADER, v_src);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, f_src);
    state->program = link_program(vs, fs);
    glDeleteShader(vs);
    glDeleteShader(fs);
    state->u_mvp = glGetUniformLocation(state->program, "u_mvp");
    state->u_color = glGetUniformLocation(state->program, "u_color");

    glGenVertexArrays(1, &state->drone_vao);
    glGenBuffers(1, &state->drone_vbo);
    glBindVertexArray(state->drone_vao);
    glBindBuffer(GL_ARRAY_BUFFER, state->drone_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(CUBE), CUBE, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    // Floor at z=0.
    constexpr float floor_quad[] = {
        -FLOOR_HALF, -FLOOR_HALF, 0.0f, FLOOR_HALF,  -FLOOR_HALF, 0.0f,
        FLOOR_HALF,  FLOOR_HALF,  0.0f, -FLOOR_HALF, -FLOOR_HALF, 0.0f,
        FLOOR_HALF,  FLOOR_HALF,  0.0f, -FLOOR_HALF, FLOOR_HALF,  0.0f,
    };

    glGenVertexArrays(1, &state->floor_vao);
    glGenBuffers(1, &state->floor_vbo);
    glBindVertexArray(state->floor_vao);
    glBindBuffer(GL_ARRAY_BUFFER, state->floor_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(floor_quad), floor_quad, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    // Grid lines on the same plane, rendered slightly above to avoid z-fighting.
    std::vector<float> grid;
    grid.reserve(6 * 2 * 1024);
    for (int i = -20; i <= 20; ++i) {
        float x = i * GRID_STEP;
        float y1 = -FLOOR_HALF;
        float y2 = FLOOR_HALF;
        grid.insert(grid.end(), {x, y1, GRID_Z, x, y2, GRID_Z});
        float y = i * GRID_STEP;
        float x1 = -FLOOR_HALF;
        float x2 = FLOOR_HALF;
        grid.insert(grid.end(), {x1, y, GRID_Z, x2, y, GRID_Z});
    }

    glGenVertexArrays(1, &state->grid_vao);
    glGenBuffers(1, &state->grid_vbo);
    glBindVertexArray(state->grid_vao);
    glBindBuffer(GL_ARRAY_BUFFER, state->grid_vbo);
    glBufferData(GL_ARRAY_BUFFER, grid.size() * sizeof(float), grid.data(), GL_STATIC_DRAW);
    state->grid_vertices = static_cast<GLsizei>(grid.size() / 3);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    state->projection = perspective(3.14159f / 3.0f, 16.0f / 9.0f, 0.1f, 100.0f);
}

Renderer::~Renderer() {
    if (state->program) glDeleteProgram(state->program);
    if (state->drone_vbo) glDeleteBuffers(1, &state->drone_vbo);
    if (state->drone_vao) glDeleteVertexArrays(1, &state->drone_vao);
    if (state->floor_vbo) glDeleteBuffers(1, &state->floor_vbo);
    if (state->floor_vao) glDeleteVertexArrays(1, &state->floor_vao);
    if (state->grid_vbo) glDeleteBuffers(1, &state->grid_vbo);
    if (state->grid_vao) glDeleteVertexArrays(1, &state->grid_vao);
}

void Renderer::draw(const GameState& s) {
    glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (s.drones.empty()) return;

    // Chase drone 0
    const Drone& d0 = s.drones[0];
    Vec<3> eye = add(d0.position, rotate(d0.orientation, CAM_OFFSET_VEC));
    Mat4 view = inverse_rigid(mul(quat_translation(d0.orientation, eye), CAM_ROTATION));

    glUseProgram(state->program);

    // Draw floor (opaque quad), then grid lines.
    Mat4 mvp = mul(state->projection, view);
    glUniformMatrix4fv(state->u_mvp, 1, GL_FALSE, mvp.data());
    static constexpr Vec<3> floor_color{0.05f, 0.05f, 0.07f};
    glUniform3fv(state->u_color, 1, floor_color.data());

    glBindVertexArray(state->floor_vao);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    static constexpr Vec<3> grid_color{0.35f, 0.35f, 0.45f};
    glUniform3fv(state->u_color, 1, grid_color.data());
    glBindVertexArray(state->grid_vao);
    glDrawArrays(GL_LINES, 0, state->grid_vertices);

    glBindVertexArray(state->drone_vao);
    static constexpr Vec<3> colors[] = {
        {1.0f, 0.3f, 0.3f},
        {0.3f, 1.0f, 0.3f},
        {0.3f, 0.3f, 1.0f},
    };

    for (size_t i = 0; i < s.drones.size(); ++i) {
        const Drone& d = s.drones[i];
        Mat4 model =
            mul(quat_translation(d.orientation, d.position), scale(1.0f, 1.0f, DRONE_Z_SCALE));
        Mat4 mvp = mul(state->projection, mul(view, model));
        glUniformMatrix4fv(state->u_mvp, 1, GL_FALSE, mvp.data());
        glUniform3fv(state->u_color, 1, colors[i % 3].data());
        glDrawArrays(GL_TRIANGLES, 0, 36);
    }
}
