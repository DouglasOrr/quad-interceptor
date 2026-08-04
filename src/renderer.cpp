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

    void upload(const void* data, GLsizeiptr bytes, GLenum usage = GL_STATIC_DRAW) const {
        glBindBuffer(GL_ARRAY_BUFFER, id);
        glBufferData(GL_ARRAY_BUFFER, bytes, data, usage);
    }
    template <typename T>
    void upload(const std::vector<T>& v, GLenum usage = GL_STATIC_DRAW) const {
        upload(v.data(), static_cast<GLsizeiptr>(v.size() * sizeof(T)), usage);
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

// A static vertex buffer + its VAO, with attribute location 0 configured as a vec3.
struct Mesh {
    struct AddCfg {
        const void* data = nullptr;
        GLsizeiptr bytes = 0;
        int loc = 0;
        int comps = 3;
        GLenum type = GL_FLOAT;
        int stride = 0;
        int offset = 0;
        bool divisor = false;
    };

    GlVertexArray vao;
    std::vector<GlBuffer> buffers;
    GLsizei vertex_count = 0;
    GLsizei instance_count = 0;
    GLenum mode = GL_TRIANGLES;

    void add(const AddCfg& c) {
        buffers.emplace_back();
        buffers.back().upload(c.data, c.bytes, GL_STATIC_DRAW);

        glBindVertexArray(vao.id);
        glBindBuffer(GL_ARRAY_BUFFER, buffers.back().id);
        glVertexAttribPointer(c.loc, c.comps, c.type, GL_FALSE, c.stride,
                              (void*)(intptr_t)c.offset);
        glEnableVertexAttribArray(c.loc);
        if (c.divisor) glVertexAttribDivisor(c.loc, 1);
        glBindVertexArray(0);
    }

    void draw() const {
        if (vertex_count <= 0) return;
        glBindVertexArray(vao.id);
        if (instance_count > 0)
            glDrawArraysInstanced(mode, 0, vertex_count, instance_count);
        else
            glDrawArrays(mode, 0, vertex_count);
    }
};

// Bundles a static cube + per-instance data (loc 1/2, divisor 1).
// Text geometry batch: builds glyph quads into a dynamic VBO.
struct TextBatch {
    GlVertexArray vao;
    GlBuffer vbo;
    GlTexture tex;
    Font font;
    std::array<int, 256> glyph_map{};
    uint glyph_cols = 0;
    uint glyph_rows = 0;
    std::vector<float> verts;

    TextBatch() {
        font = Font::load();
        glyph_map.fill(-1);
        for (size_t gi = 0; gi < font.glyphs.size(); ++gi)
            glyph_map[font.glyphs[gi].ch] = int(gi);
        while (glyph_cols * glyph_cols < font.glyphs.size())
            ++glyph_cols;
        glyph_rows = (font.glyphs.size() + glyph_cols - 1) / glyph_cols;

        const uint sheet_w = glyph_cols * font.width;
        const uint sheet_h = glyph_rows * font.height;
        std::vector<uint8_t> sheet(sheet_w * sheet_h, 0);
        for (size_t gi = 0; gi < font.glyphs.size(); ++gi) {
            const auto& data = font.glyphs[gi].data;
            const uint col = uint(gi) % glyph_cols;
            const uint row = uint(gi) / glyph_cols;
            for (uint y = 0; y < font.height; ++y)
                for (uint x = 0; x < font.width; ++x)
                    sheet[(row * font.height + y) * sheet_w + col * font.width + x] =
                        data[y * font.width + x];
        }
        glBindTexture(GL_TEXTURE_2D, tex.id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, sheet_w, sheet_h, 0, GL_RED, GL_UNSIGNED_BYTE,
                     sheet.data());
        glBindTexture(GL_TEXTURE_2D, 0);

        glBindVertexArray(vao.id);
        glBindBuffer(GL_ARRAY_BUFFER, vbo.id);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                              (void*)(2 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
    }

    void draw(float x, float top_y, float scale, const char* text) {
        const float cell_w = font.width * scale;
        const float cell_h = font.height * scale;
        const float advance = font.width * scale;
        verts.clear();
        std::array<std::array<float, 4>, 4> quad{};
        for (size_t ti = 0; text[ti] != '\0'; ++ti) {
            const int i = glyph_map[static_cast<unsigned char>(text[ti])];
            if (i < 0) continue;
            const uint col = uint(i) % glyph_cols;
            const uint row = uint(i) / glyph_cols;
            const float x0 = x + float(ti) * advance;
            const float x1 = x0 + cell_w;
            const float u0 = float(col) / float(glyph_cols);
            const float u1 = float(col + 1) / float(glyph_cols);
            const float vtop = float(row) / float(glyph_rows);
            const float vbot = float(row + 1) / float(glyph_rows);
            quad[0] = {x0, top_y + cell_h, u0, vtop};
            quad[1] = {x1, top_y + cell_h, u1, vtop};
            quad[2] = {x1, top_y, u1, vbot};
            quad[3] = {x0, top_y, u0, vbot};
            const int tris[6] = {0, 1, 2, 0, 2, 3};
            for (int tri : tris) {
                auto& q = quad[tri];
                verts.insert(verts.end(), {q[0], q[1], q[2], q[3]});
            }
        }
        if (verts.empty()) return;
        glBindBuffer(GL_ARRAY_BUFFER, vbo.id);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(verts.size() * sizeof(float)),
                     verts.data(), GL_DYNAMIC_DRAW);
        glBindVertexArray(vao.id);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, tex.id);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(verts.size() / 4));
        glBindVertexArray(0);
        glBindTexture(GL_TEXTURE_2D, 0);
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

    Mesh drone;
    Mesh floor;
    Mesh grid;

    Mesh city;
    int64 city_anchor_x = INT64_MAX;
    int64 city_anchor_y = INT64_MAX;

    TextBatch text;

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
    state->drone.vertex_count = 36;
    state->drone.mode = GL_TRIANGLES;
    state->drone.add({.data = CUBE, .bytes = sizeof(CUBE), .loc = 0, .comps = 3});

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

    state->city.mode = GL_TRIANGLES;
    state->city.vertex_count = 36;
    state->city.instance_count = 0;
    state->city.add({.data = CUBE, .bytes = sizeof(CUBE), .loc = 0, .comps = 3});
    state->city.add({.data = nullptr, .bytes = 0, .loc = 1, .comps = 3, .divisor = true});
    state->city.add({.data = nullptr, .bytes = 0, .loc = 2, .comps = 3, .divisor = true});

    // Floor at z=0.
    constexpr float floor_quad[] = {
        -FLOOR_HALF, -FLOOR_HALF, 0.0f, FLOOR_HALF,  -FLOOR_HALF, 0.0f,
        FLOOR_HALF,  FLOOR_HALF,  0.0f, -FLOOR_HALF, -FLOOR_HALF, 0.0f,
        FLOOR_HALF,  FLOOR_HALF,  0.0f, -FLOOR_HALF, FLOOR_HALF,  0.0f,
    };
    state->floor.vertex_count = 6;
    state->floor.mode = GL_TRIANGLES;
    state->floor.add({.data = floor_quad, .bytes = sizeof(floor_quad), .loc = 0, .comps = 3});

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
    state->grid.vertex_count = static_cast<GLsizei>(grid.size() / 3);
    state->grid.mode = GL_LINES;
    state->grid.add({.data = grid.data(),
                     .bytes = static_cast<GLsizeiptr>(grid.size() * sizeof(float)),
                     .loc = 0,
                     .comps = 3});

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


    state->projection = mat_perspective(FOV_DEGS * (M_PIf / 180.0f), 16.0f / 9.0f, 0.1f, 1000.0f);

    // Default state
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glDisable(GL_BLEND);
}

Renderer::~Renderer() = default;

void Renderer::draw(const GameState& s, float fps) {
    glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (s.drones.empty()) return;

    // Chase drone 0
    const Drone& d0 = s.drones[0];
    Vec<3> eye = d0.position + rotate(d0.orientation, CAM_OFFSET_VEC);
    Mat4 view = rigid_inverse(mul(mat_from(d0.orientation, eye), CAM_ROTATION));

    // Draw floor (opaque quad), then grid lines.
    state->program.use();
    Mat4 mvp = mul(state->projection, view);
    state->u_mvp.set(mvp);
    state->u_color.set(Vec<3>{0.05f, 0.05f, 0.07f});
    state->floor.draw();
    state->u_color.set(Vec<3>{0.35f, 0.35f, 0.45f});
    state->grid.draw();

    // City: build one big Mesh with per-vertex positions/colors
    const City& city = s.city;
    const int64 ax = static_cast<int64>(std::floor(d0.position[0] / float(city.cell_size)));
    const int64 ay = static_cast<int64>(std::floor(d0.position[1] / float(city.cell_size)));
    if (ax != state->city_anchor_x || ay != state->city_anchor_y) {
        state->city_anchor_x = ax;
        state->city_anchor_y = ay;
        std::vector<float> inst_pos, inst_color;
        for (int64 dy = -CITY_RADIUS; dy <= CITY_RADIUS; ++dy) {
            for (int64 dx = -CITY_RADIUS; dx <= CITY_RADIUS; ++dx) {
                Cell c = city(ax + dx, ay + dy);
                if (c.height <= 0.0f) continue;
                float k = float((c.hash >> 32) & 0xff) / 255.0f;
                float shade = 0.2f + 0.2f * k;
                float wx = float(ax + dx) * float(city.cell_size);
                float wy = float(ay + dy) * float(city.cell_size);
                inst_pos.insert(inst_pos.end(), {wx, wy, c.height});
                inst_color.insert(inst_color.end(), {shade, shade, shade});
            }
        }
        state->city.instance_count = static_cast<GLsizei>(inst_pos.size() / 3);
        state->city.buffers[1].upload(inst_pos, GL_DYNAMIC_DRAW);
        state->city.buffers[2].upload(inst_color, GL_DYNAMIC_DRAW);
    }
    if (state->city.instance_count > 0) {
        state->city_program.use();
        state->u_city_mvp.set(mvp);
        state->u_city_cell_size.set(float(city.cell_size));
        state->u_city_light_dir.set(LIGHT_DIR);
        state->u_city_cam_pos.set(eye);
        state->city.draw();
    }

    // Draw drones
    state->program.use();
    for (size_t i = 0; i < s.drones.size(); ++i) {
        const Drone& d = s.drones[i];
        Mat4 model = mul(mat_from(d.orientation, d.position), mat_diag(1, 1, Drone::height, 1));
        Mat4 mvp = mul(state->projection, mul(view, model));
        state->u_mvp.set(mvp);
        state->u_color.set(i == 0 ? Vec<3>{0.3f, 0.3f, 0.8f} : Vec<3>{0.8f, 0.3f, 0.3f});
        state->drone.draw();
    }

    // Draw FPS counter
    char fps_label[32];
    std::snprintf(fps_label, sizeof(fps_label), "%d FPS", int(fps));

    const float scale = 1.5f;
    const float margin = 8.0f;
    GLint vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);
    const float vp_width = float(vp[2]);
    const float vp_height = float(vp[3]);
    const float text_w = float(std::strlen(fps_label)) * state->text.font.width * scale;
    const float x = vp_width - margin - text_w;
    const float top_y = vp_height - margin - state->text.font.height * scale;

    state->text_program.use();
    state->u_text_mvp.set(mat_ortho(0.0f, vp_width, 0.0f, vp_height));
    state->u_text_color.set(Vec<3>{1.0f, 0.4f, 0.4f});
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    state->text.draw(x, top_y, scale, fps_label);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}
