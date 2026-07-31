#include "game.hpp"
#include "platform.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

////////////////////////////////////////////////////////////////////////////////
// RAII OpenGL

struct GlBuffer {
    GLuint id = 0;

    GlBuffer() { glGenBuffers(1, &id); }
    ~GlBuffer() {
        if (id) glDeleteBuffers(1, &id);
    }
    GlBuffer(const GlBuffer&) = delete;
    GlBuffer& operator=(const GlBuffer&) = delete;
    GlBuffer(GlBuffer&& o) noexcept : id(o.id) { o.id = 0; }
    GlBuffer& operator=(GlBuffer&& o) noexcept {
        if (this != &o) std::swap(id, o.id);
        return *this;
    }
};

struct GlVertexArray {
    GLuint id = 0;

    GlVertexArray() { glGenVertexArrays(1, &id); }
    ~GlVertexArray() {
        if (id) glDeleteVertexArrays(1, &id);
    }
    GlVertexArray(const GlVertexArray&) = delete;
    GlVertexArray& operator=(const GlVertexArray&) = delete;
    GlVertexArray(GlVertexArray&& o) noexcept : id(o.id) { o.id = 0; }
    GlVertexArray& operator=(GlVertexArray&& o) noexcept {
        if (this != &o) std::swap(id, o.id);
        return *this;
    }
};

// Typed uniform location. Setting an unset (-1) location is a no-op.
struct Uniform {
    GLint loc = -1;

    void set(const Mat4& m) const {
        if (loc >= 0) glUniformMatrix4fv(loc, 1, GL_FALSE, m.data);
    }
    void set(const Vec<3>& v) const {
        if (loc >= 0) glUniform3fv(loc, 1, v.data);
    }
};

struct GlProgram {
    GLuint id = 0;

    GlProgram() = default;
    GlProgram(const char* v_src, const char* f_src) {
        GLuint vs = compile(GL_VERTEX_SHADER, v_src);
        GLuint fs = compile(GL_FRAGMENT_SHADER, f_src);
        id = glCreateProgram();
        glAttachShader(id, vs);
        glAttachShader(id, fs);
        glLinkProgram(id);
        GLint ok;
        glGetProgramiv(id, GL_LINK_STATUS, &ok);
        glDeleteShader(vs);
        glDeleteShader(fs);
        if (!ok) {
            char log[512];
            glGetProgramInfoLog(id, 512, nullptr, log);
            glDeleteProgram(id);
            id = 0;
            throw std::runtime_error(std::string("program link: ") + log);
        }
    }
    ~GlProgram() {
        if (id) glDeleteProgram(id);
    }
    GlProgram(const GlProgram&) = delete;
    GlProgram& operator=(const GlProgram&) = delete;
    GlProgram(GlProgram&& o) noexcept : id(o.id) { o.id = 0; }
    GlProgram& operator=(GlProgram&& o) noexcept {
        if (this != &o) std::swap(id, o.id);
        return *this;
    }

    void use() const { glUseProgram(id); }
    Uniform uniform(const char* name) const { return {glGetUniformLocation(id, name)}; }

private:
    static GLuint compile(GLenum type, const char* src) {
        GLuint s = glCreateShader(type);
        glShaderSource(s, 1, &src, nullptr);
        glCompileShader(s);
        GLint ok;
        glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[512];
            glGetShaderInfoLog(s, 512, nullptr, log);
            glDeleteShader(s);
            throw std::runtime_error(std::string("shader compile: ") + log + "\n--- source ---\n" +
                                     src);
        }
        return s;
    }
};

// A static vertex buffer + its VAO, with attribute location 0 configured as a
// vec3. Bundles the vertex count so draw() needs no extra arguments.
struct Mesh {
    GlVertexArray vao;
    GlBuffer vbo;
    GLsizei count = 0;

    Mesh() = default;
    Mesh(const void* data, GLsizeiptr bytes, GLsizei count) : count(count) {
        glBindVertexArray(vao.id);
        glBindBuffer(GL_ARRAY_BUFFER, vbo.id);
        glBufferData(GL_ARRAY_BUFFER, bytes, data, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }

    void draw(GLenum mode) const {
        glBindVertexArray(vao.id);
        glDrawArrays(mode, 0, count);
    }
};

////////////////////////////////////////////////////////////////////////////////
// Renderer impl

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
    GlProgram program;
    Uniform u_mvp;
    Uniform u_color;

    Mesh drone;
    Mesh floor;
    Mesh grid;

    Mat4 projection;
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

    state->program = GlProgram(v_src, f_src);
    state->u_mvp = state->program.uniform("u_mvp");
    state->u_color = state->program.uniform("u_color");
    state->drone = Mesh(CUBE, sizeof(CUBE), 36);

    // Floor at z=0.
    constexpr float floor_quad[] = {
        -FLOOR_HALF, -FLOOR_HALF, 0.0f, FLOOR_HALF,  -FLOOR_HALF, 0.0f,
        FLOOR_HALF,  FLOOR_HALF,  0.0f, -FLOOR_HALF, -FLOOR_HALF, 0.0f,
        FLOOR_HALF,  FLOOR_HALF,  0.0f, -FLOOR_HALF, FLOOR_HALF,  0.0f,
    };
    state->floor = Mesh(floor_quad, sizeof(floor_quad), 6);

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
    state->grid =
        Mesh(grid.data(), grid.size() * sizeof(float), static_cast<GLsizei>(grid.size() / 3));

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    state->projection = mat_perspective(3.14159f / 3.0f, 16.0f / 9.0f, 0.1f, 100.0f);
}

Renderer::~Renderer() = default;

void Renderer::draw(const GameState& s) {
    glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (s.drones.empty()) return;

    state->program.use();

    // Chase drone 0
    const Drone& d0 = s.drones[0];
    Vec<3> eye = add(d0.position, rotate(d0.orientation, CAM_OFFSET_VEC));
    Mat4 view = rigid_inverse(mul(mat_from(d0.orientation, eye), CAM_ROTATION));

    // Draw floor (opaque quad), then grid lines.
    Mat4 mvp = mul(state->projection, view);
    state->u_mvp.set(mvp);
    state->u_color.set(Vec<3>{0.05f, 0.05f, 0.07f});
    state->floor.draw(GL_TRIANGLES);

    state->u_color.set(Vec<3>{0.35f, 0.35f, 0.45f});
    state->grid.draw(GL_LINES);

    static constexpr Vec<3> colors[] = {
        {1.0f, 0.3f, 0.3f},
        {0.3f, 1.0f, 0.3f},
        {0.3f, 0.3f, 1.0f},
    };

    for (size_t i = 0; i < s.drones.size(); ++i) {
        const Drone& d = s.drones[i];
        Mat4 model = mul(mat_from(d.orientation, d.position), mat_scale(1.0f, 1.0f, DRONE_Z_SCALE));
        Mat4 mvp = mul(state->projection, mul(view, model));
        state->u_mvp.set(mvp);
        state->u_color.set(colors[i % 3]);
        state->drone.draw(GL_TRIANGLES);
    }
}
