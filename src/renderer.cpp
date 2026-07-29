#include "game.hpp"
#include "platform.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace {

using Mat4 = std::array<float, 16>;  // column-major: index = i44(row, col)

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

Mat4 look_at(Vec<3> eye, Vec<3> center, Vec<3> up) {
    float fx = center[0] - eye[0], fy = center[1] - eye[1], fz = center[2] - eye[2];
    float fl = std::sqrt(fx * fx + fy * fy + fz * fz);
    fx /= fl;
    fy /= fl;
    fz /= fl;
    float sx = fy * up[2] - fz * up[1];
    float sy = fz * up[0] - fx * up[2];
    float sz = fx * up[1] - fy * up[0];
    float sl = std::sqrt(sx * sx + sy * sy + sz * sz);
    sx /= sl;
    sy /= sl;
    sz /= sl;
    float ux = sy * fz - sz * fy;
    float uy = sz * fx - sx * fz;
    float uz = sx * fy - sy * fx;
    Mat4 m{};
    m[i44(0, 0)] = sx;
    m[i44(0, 1)] = sy;
    m[i44(0, 2)] = sz;
    m[i44(1, 0)] = ux;
    m[i44(1, 1)] = uy;
    m[i44(1, 2)] = uz;
    m[i44(2, 0)] = -fx;
    m[i44(2, 1)] = -fy;
    m[i44(2, 2)] = -fz;
    m[i44(0, 3)] = -(sx * eye[0] + sy * eye[1] + sz * eye[2]);
    m[i44(1, 3)] = -(ux * eye[0] + uy * eye[1] + uz * eye[2]);
    m[i44(2, 3)] = fx * eye[0] + fy * eye[1] + fz * eye[2];
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
    GLuint vao = 0;
    GLuint vbo = 0;
    GLint u_mvp = -1;
    GLint u_color = -1;
    Mat4 projection{};
    Mat4 view{};
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

    glGenVertexArrays(1, &state->vao);
    glGenBuffers(1, &state->vbo);
    glBindVertexArray(state->vao);
    glBindBuffer(GL_ARRAY_BUFFER, state->vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(CUBE), CUBE, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);

    state->projection = perspective(3.14159f / 3.0f, 16.0f / 9.0f, 0.1f, 100.0f);
    state->view = look_at({0.0f, -10.0f, 5.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f});
}

Renderer::~Renderer() {
    if (state->program) glDeleteProgram(state->program);
    if (state->vbo) glDeleteBuffers(1, &state->vbo);
    if (state->vao) glDeleteVertexArrays(1, &state->vao);
}

void Renderer::draw(const GameState& s) {
    glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(state->program);
    glBindVertexArray(state->vao);

    static constexpr Vec<3> colors[] = {
        {1.0f, 0.3f, 0.3f},
        {0.3f, 1.0f, 0.3f},
        {0.3f, 0.3f, 1.0f},
    };

    for (size_t i = 0; i < s.drones.size(); ++i) {
        const Drone& d = s.drones[i];
        Mat4 model = quat_translation(d.orientation, d.position);
        Mat4 mvp = mul(state->projection, mul(state->view, model));
        glUniformMatrix4fv(state->u_mvp, 1, GL_FALSE, mvp.data());
        glUniform3fv(state->u_color, 1, colors[i % 3].data());
        glDrawArrays(GL_TRIANGLES, 0, 36);
    }
}
