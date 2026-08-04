#include "game.hpp"
#include "platform.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

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

struct GlTexture {
    GLuint id = 0;

    GlTexture() { glGenTextures(1, &id); }
    ~GlTexture() {
        if (id) glDeleteTextures(1, &id);
    }
    GlTexture(const GlTexture&) = delete;
    GlTexture& operator=(const GlTexture&) = delete;
    GlTexture(GlTexture&& o) noexcept : id(o.id) { o.id = 0; }
    GlTexture& operator=(GlTexture&& o) noexcept {
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
    void set(float f) const {
        if (loc >= 0) glUniform1f(loc, f);
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
constexpr float FOV_DEGS = 100.0f;
constexpr float FLOOR_HALF = 500.0f;  // floor spans +/-FLOOR_HALF on X/Y
constexpr float GRID_STEP = 3.0f;
constexpr int CITY_RADIUS = 60;   // cells rendered around the player in each direction
constexpr float GRID_Z = 0.001f;  // slightly above floor to avoid z-fighting
constexpr Vec<3> LIGHT_DIR{0.40f, 0.55f, 0.73f};  // world-space direction toward the sun
// constexpr Vec<3> CAM_OFFSET_VEC{0.5f, 0.0f, 0.0f};  // front (cockpit)
constexpr Vec<3> CAM_OFFSET_VEC{-1.2f, 0.0f, 0.5f};  // chase (behind, slightly up)

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

    GlProgram city_program;
    Uniform u_city_mvp;
    Uniform u_city_cell_size;
    Uniform u_city_light_dir;
    Uniform u_city_cam_pos;

    GlProgram text_program;
    Uniform u_text_mvp;
    Uniform u_text_color;
    GlTexture font_tex;
    GlVertexArray text_vao;
    GlBuffer text_vbo;
    std::vector<float> text_verts;
    Font font;
    uint glyph_cols = 0;
    uint glyph_rows = 0;
    std::array<int, 256> glyph_map;  // byte value -> glyph index, or -1

    Mesh drone;
    Mesh floor;
    Mesh grid;

    GlVertexArray city_vao;
    GlBuffer city_cube_vbo;
    GlBuffer city_inst_vbo;
    std::vector<float> city_inst;
    GLsizei city_count = 0;
    int64 city_anchor_x = INT64_MAX;
    int64 city_anchor_y = INT64_MAX;

    Mat4 projection;
};

Renderer::Renderer() : state(std::make_unique<State>()) {
    const char* v_src = R"(
#version 330 core
layout(location=0) in vec3 pos;
uniform mat4 u_mvp;
void main(){ gl_Position = u_mvp * vec4(pos, 1.0); }
)";
    const char* f_src = R"(
#version 330 core
out vec4 frag;
uniform vec3 u_color;
void main(){ frag = vec4(u_color, 1.0); }
)";

    state->program = GlProgram(v_src, f_src);
    state->u_mvp = state->program.uniform("u_mvp");
    state->u_color = state->program.uniform("u_color");
    state->drone = Mesh(CUBE, sizeof(CUBE), 36);

    // Per-instance city program: each instance carries the cell corner world x/y + height
    // (loc 1) and a grey shade (loc 2); the unit cube is scaled/raised from that corner.
    const char* city_v_src = R"(
#version 330 core
layout(location=0) in vec3 pos;
layout(location=1) in vec3 inst_world;   // x, y, height
layout(location=2) in vec3 inst_color;
uniform mat4 u_mvp;
uniform float u_city_cell_size;
out vec3 v_world;
out vec3 v_color;
void main(){
  vec3 scale = vec3(u_city_cell_size, u_city_cell_size, inst_world.z);
  vec3 world = vec3(inst_world.x, inst_world.y, 0.0);
  vec3 p = world + scale * (pos + 0.5);
  gl_Position = u_mvp * vec4(p, 1.0);
  v_world = p;
  v_color = inst_color;
}
)";
    const char* city_f_src = R"(
#version 330 core
in vec3 v_world;
in vec3 v_color;
out vec4 frag;
uniform vec3 u_light_dir;
uniform vec3 u_cam_pos;
const vec3 u_ambient = vec3(0.22);
const vec3 u_diffuse = vec3(0.70);
const vec3 u_specular = vec3(0.30);
const float u_shininess = 32.0;
void main(){
  // Flat per-face normal, recovered from fragment derivatives of world position.
  vec3 N = normalize(cross(dFdx(v_world), dFdy(v_world)));
  vec3 L = normalize(u_light_dir);
  vec3 V = normalize(u_cam_pos - v_world);
  vec3 H = normalize(L + V);
  float diff = max(dot(N, L), 0.0);
  float spec = diff > 0.0 ? pow(max(dot(N, H), 0.0), u_shininess) : 0.0;
  frag = vec4(v_color * (u_ambient + u_diffuse * diff) + u_specular * spec, 1.0);
}
)";
    state->city_program = GlProgram(city_v_src, city_f_src);
    state->u_city_mvp = state->city_program.uniform("u_mvp");
    state->u_city_cell_size = state->city_program.uniform("u_city_cell_size");
    state->u_city_light_dir = state->city_program.uniform("u_light_dir");
    state->u_city_cam_pos = state->city_program.uniform("u_cam_pos");

    // City instanced draw setup: cube geometry at loc 0, per-instance data at 1 & 2.
    glBindBuffer(GL_ARRAY_BUFFER, state->city_cube_vbo.id);
    glBufferData(GL_ARRAY_BUFFER, sizeof(CUBE), CUBE, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, state->city_inst_vbo.id);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

    glBindVertexArray(state->city_vao.id);
    glBindBuffer(GL_ARRAY_BUFFER, state->city_cube_vbo.id);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, state->city_inst_vbo.id);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2, 1);
    glBindVertexArray(0);

    // Floor at z=0.
    constexpr float floor_quad[] = {
        -FLOOR_HALF, -FLOOR_HALF, 0.0f, FLOOR_HALF,  -FLOOR_HALF, 0.0f,
        FLOOR_HALF,  FLOOR_HALF,  0.0f, -FLOOR_HALF, -FLOOR_HALF, 0.0f,
        FLOOR_HALF,  FLOOR_HALF,  0.0f, -FLOOR_HALF, FLOOR_HALF,  0.0f,
    };
    state->floor = Mesh(floor_quad, sizeof(floor_quad), 6);

    // Grid lines on the same plane, rendered slightly above to avoid z-fighting.
    std::vector<float> grid;
    auto n_half = int(FLOOR_HALF / GRID_STEP);
    for (int i = -n_half; i <= n_half; ++i) {
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

    // Text: texture-sampled glyph quads onto an orthographic overlay.
    const char* text_v_src = R"(
#version 330 core
layout(location=0) in vec2 pos;
layout(location=1) in vec2 uv;
uniform mat4 u_mvp;
out vec2 v_uv;
void main(){ gl_Position = u_mvp * vec4(pos, 0.0, 1.0); v_uv = uv; }
)";
    const char* text_f_src = R"(
#version 330 core
in vec2 v_uv;
uniform sampler2D u_tex;
uniform vec3 u_color;
out vec4 frag;
void main(){ float a = texture(u_tex, v_uv).r; frag = vec4(u_color, a); }
)";
    state->text_program = GlProgram(text_v_src, text_f_src);
    state->u_text_mvp = state->text_program.uniform("u_mvp");
    state->u_text_color = state->text_program.uniform("u_color");

    // Unpack the 1-bit atlas into a sheet texture (one glyph per cell).
    state->font = Font::load();
    const Font& font = state->font;
    state->glyph_map.fill(-1);
    for (size_t gi = 0; gi < font.glyphs.size(); ++gi)
        state->glyph_map[font.glyphs[gi].ch] = int(gi);
    while (state->glyph_cols * state->glyph_cols < font.glyphs.size())
        ++state->glyph_cols;
    state->glyph_rows = (font.glyphs.size() + state->glyph_cols - 1) / state->glyph_cols;
    const uint sheet_w = state->glyph_cols * font.width;
    const uint sheet_h = state->glyph_rows * font.height;
    std::vector<uint8_t> sheet(sheet_w * sheet_h, 0);
    for (size_t gi = 0; gi < font.glyphs.size(); ++gi) {
        const std::vector<uint8_t>& data = font.glyphs[gi].data;
        const uint cell_col = uint(gi) % state->glyph_cols;
        const uint cell_row = uint(gi) / state->glyph_cols;
        for (uint y = 0; y < font.height; ++y) {
            for (uint x = 0; x < font.width; ++x) {
                sheet[(cell_row * font.height + y) * sheet_w + cell_col * font.width + x] =
                    data[y * font.width + x];
            }
        }
    }
    glBindTexture(GL_TEXTURE_2D, state->font_tex.id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, sheet_w, sheet_h, 0, GL_RED, GL_UNSIGNED_BYTE,
                 sheet.data());
    glBindTexture(GL_TEXTURE_2D, 0);

    glBindVertexArray(state->text_vao.id);
    glBindBuffer(GL_ARRAY_BUFFER, state->text_vbo.id);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    state->projection = mat_perspective(FOV_DEGS * (M_PIf / 180.0f), 16.0f / 9.0f, 0.1f, 1000.0f);
}

Renderer::~Renderer() = default;

void Renderer::draw(const GameState& s, float fps) {
    glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (s.drones.empty()) return;

    state->program.use();

    // Chase drone 0
    const Drone& d0 = s.drones[0];
    Vec<3> eye = d0.position + rotate(d0.orientation, CAM_OFFSET_VEC);
    Mat4 view = rigid_inverse(mul(mat_from(d0.orientation, eye), CAM_ROTATION));

    // Draw floor (opaque quad), then grid lines.
    Mat4 mvp = mul(state->projection, view);
    state->u_mvp.set(mvp);
    state->u_color.set(Vec<3>{0.05f, 0.05f, 0.07f});
    state->floor.draw(GL_TRIANGLES);

    state->u_color.set(Vec<3>{0.35f, 0.35f, 0.45f});
    state->grid.draw(GL_LINES);

    // City: instance the visible cells in a single draw call. Refill the instance
    // buffer only when the player crosses into a new anchor cell.
    const City& city = s.city;
    const int64 ax = static_cast<int64>(std::floor(d0.position[0] / float(city.cell_size)));
    const int64 ay = static_cast<int64>(std::floor(d0.position[1] / float(city.cell_size)));
    if (ax != state->city_anchor_x || ay != state->city_anchor_y) {
        state->city_anchor_x = ax;
        state->city_anchor_y = ay;
        state->city_inst.clear();
        for (int64 dy = -CITY_RADIUS; dy <= CITY_RADIUS; ++dy) {
            for (int64 dx = -CITY_RADIUS; dx <= CITY_RADIUS; ++dx) {
                Cell c = city(ax + dx, ay + dy);
                if (c.height <= 0.0f) continue;  // street
                float k = float((c.hash >> 32) & 0xff) / 255.0f;
                float shade = 0.2f + 0.2f * k;
                float wx = float(ax + dx) * float(city.cell_size);
                float wy = float(ay + dy) * float(city.cell_size);
                state->city_inst.insert(state->city_inst.end(),
                                        {wx, wy, c.height, shade, shade, shade});
            }
        }
        state->city_count = static_cast<GLsizei>(state->city_inst.size() / 6);
        if (state->city_count > 0) {
            glBindBuffer(GL_ARRAY_BUFFER, state->city_inst_vbo.id);
            glBufferData(GL_ARRAY_BUFFER,
                         static_cast<GLsizeiptr>(state->city_inst.size() * sizeof(float)),
                         state->city_inst.data(), GL_DYNAMIC_DRAW);
        }
    }
    if (state->city_count > 0) {
        state->city_program.use();
        state->u_city_mvp.set(mvp);
        state->u_city_cell_size.set(float(city.cell_size));
        state->u_city_light_dir.set(LIGHT_DIR);
        state->u_city_cam_pos.set(eye);
        glBindVertexArray(state->city_vao.id);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 36, state->city_count);
        glBindVertexArray(0);
        state->program.use();
    }

    for (size_t i = 0; i < s.drones.size(); ++i) {
        const Drone& d = s.drones[i];
        Mat4 model = mul(mat_from(d.orientation, d.position), mat_diag(1, 1, Drone::height, 1));
        Mat4 mvp = mul(state->projection, mul(view, model));
        state->u_mvp.set(mvp);
        state->u_color.set(i == 0 ? Vec<3>{0.3f, 0.3f, 0.8f} : Vec<3>{0.8f, 0.3f, 0.3f});
        state->drone.draw(GL_TRIANGLES);
    }

    auto draw_text = [this](float x, float top_y, float scale, const char* text, Vec<3> color) {
        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        const float vp_width = float(vp[2]);
        const float vp_height = float(vp[3]);

        const Font& font = state->font;
        const float cell_w = font.width * scale;
        const float cell_h = font.height * scale;
        const float advance = font.width * scale;
        state->text_verts.clear();
        std::array<std::array<float, 4>, 4> quad{};  // (x, y, u, v)
        for (size_t ti = 0; text[ti] != '\0'; ++ti) {
            const int i = state->glyph_map[static_cast<unsigned char>(text[ti])];
            if (i < 0) continue;
            const uint col = uint(i) % state->glyph_cols;
            const uint row = uint(i) / state->glyph_cols;
            const float x0 = x + float(ti) * advance;
            const float x1 = x0 + cell_w;
            const float u0 = float(col) / float(state->glyph_cols);
            const float u1 = float(col + 1) / float(state->glyph_cols);
            const float vtop = float(row) / float(state->glyph_rows);
            const float vbot = float(row + 1) / float(state->glyph_rows);
            quad[0] = {x0, top_y + cell_h, u0, vtop};
            quad[1] = {x1, top_y + cell_h, u1, vtop};
            quad[2] = {x1, top_y, u1, vbot};
            quad[3] = {x0, top_y, u0, vbot};
            const int tris[6] = {0, 1, 2, 0, 2, 3};
            for (int tri : tris) {
                auto& q = quad[tri];
                state->text_verts.insert(state->text_verts.end(), {q[0], q[1], q[2], q[3]});
            }
        }
        if (state->text_verts.empty()) return;

        glBindBuffer(GL_ARRAY_BUFFER, state->text_vbo.id);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(state->text_verts.size() * sizeof(float)),
                     state->text_verts.data(), GL_DYNAMIC_DRAW);

        state->text_program.use();
        state->u_text_mvp.set(mat_ortho(0.0f, vp_width, 0.0f, vp_height));
        state->u_text_color.set(color);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, state->font_tex.id);

        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBindVertexArray(state->text_vao.id);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(state->text_verts.size() / 4));
        glBindVertexArray(0);
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);

        state->program.use();
        glBindTexture(GL_TEXTURE_2D, 0);
    };

    char label[32];
    std::snprintf(label, sizeof(label), "%d FPS", int(fps));

    // Right-align to the top-right corner.
    const float scale = 1.5f;
    const float margin = 8.0f;
    GLint vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);
    const float vp_width = float(vp[2]);
    const float vp_height = float(vp[3]);
    const float text_w = float(std::strlen(label)) * state->font.width * scale;
    const float x = vp_width - margin - text_w;
    const float top_y = vp_height - margin - state->font.height * scale;
    draw_text(x, top_y, scale, label, Vec<3>{0.7f, 0.8f, 1.0f});
}
