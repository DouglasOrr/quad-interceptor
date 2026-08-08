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

std::vector<Vec<3>> make_cube() {
    std::vector<Vec<3>> v;
    // 6 faces, each a unit square in the two axes perpendicular to its normal.
    const Vec<3> corners[8] = {{-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, -0.5f},
                               {-0.5f, 0.5f, -0.5f},  {-0.5f, -0.5f, 0.5f}, {0.5f, -0.5f, 0.5f},
                               {0.5f, 0.5f, 0.5f},    {-0.5f, 0.5f, 0.5f}};
    const int faces[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4},
                             {2, 3, 7, 6}, {1, 2, 6, 5}, {3, 0, 4, 7}};
    for (const auto& f : faces) {
        v.push_back(corners[f[0]]);
        v.push_back(corners[f[1]]);
        v.push_back(corners[f[2]]);
        v.push_back(corners[f[0]]);
        v.push_back(corners[f[2]]);
        v.push_back(corners[f[3]]);
    }
    return v;
}

std::vector<Vec<3>> make_cylinder(uint steps) {
    std::vector<Vec<3>> v;
    for (auto i = 0u; i < steps; ++i) {
        float a0 = 2.0f * M_PIf * float(i) / float(steps);
        float a1 = 2.0f * M_PIf * float(i + 1) / float(steps);
        float x0 = 0.5f * std::cos(a0), y0 = 0.5f * std::sin(a0);
        float x1 = 0.5f * std::cos(a1), y1 = 0.5f * std::sin(a1);
        v.push_back({0, 0, 0.5f});
        v.push_back({x0, y0, 0.5f});
        v.push_back({x1, y1, 0.5f});
        v.push_back({0, 0, -0.5f});
        v.push_back({x1, y1, -0.5f});
        v.push_back({x0, y0, -0.5f});
        v.push_back({x0, y0, -0.5f});
        v.push_back({x0, y0, 0.5f});
        v.push_back({x1, y1, -0.5f});
        v.push_back({x1, y1, -0.5f});
        v.push_back({x0, y0, 0.5f});
        v.push_back({x1, y1, 0.5f});
    }
    return v;
}

void append_vertices(std::vector<float>& dest,
                     const std::vector<Vec<3>>& src,
                     const std::optional<Mat4>& m = std::nullopt) {
    for (auto p : src) {
        Vec<4> v{p[0], p[1], p[2], 1.0f};
        if (m) v = mul(*m, v);
        dest.push_back(v[0]);
        dest.push_back(v[1]);
        dest.push_back(v[2]);
    }
}

const char* PHONG_F_SRC = R"(
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
  vec3 N = normalize(cross(dFdx(v_world), dFdy(v_world)));
  vec3 L = normalize(u_light_dir);
  vec3 V = normalize(u_cam_pos - v_world);
  vec3 H = normalize(L + V);
  float diff = max(dot(N, L), 0.0);
  float spec = diff > 0.0 ? pow(max(dot(N, H), 0.0), u_shininess) : 0.0;
  frag = vec4(v_color * (u_ambient + u_diffuse * diff) + u_specular * spec, 1.0);
}
)";

// unit cube (centered at origin, side length 1)
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
// 2D unit quad (top-left origin)
constexpr float QUAD2[] = {
    0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f,
};
// 2D unit quad centered at origin, in [-0.5,0.5] (for instanced street bands).
constexpr float QUAD2C[] = {
    -0.5f, -0.5f, 0.5f, -0.5f, 0.5f, 0.5f, -0.5f, -0.5f, 0.5f, 0.5f, -0.5f, 0.5f,
};
// 3D unit quad in the z=0 plane (centered at origin, like CUBE), used for floor tiles.
constexpr float QUAD3[] = {
    -0.5f, -0.5f, 0.0f, 0.5f, -0.5f, 0.0f, 0.5f,  0.5f, 0.0f,
    -0.5f, -0.5f, 0.0f, 0.5f, 0.5f,  0.0f, -0.5f, 0.5f, 0.0f,
};

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
    void set(const Vec<2>& v) const {
        if (loc >= 0) glUniform2fv(loc, 1, v.data);
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
    Uniform uniform(const char* name) const {
        GLint loc = glGetUniformLocation(id, name);
        if (loc < 0)
            throw std::runtime_error(std::string("uniform '") + name +
                                     "' not found in shader program");
        return {loc};
    }

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

// Bundles a static quad + per-instance data (pos, glyph) for instanced text rendering.
struct TextBatch {
    GlTexture tex;
    Font font;
    std::array<int, 256> glyph_map{};
    uint glyph_cols = 0;
    uint glyph_rows = 0;
    Mesh mesh;
    GlProgram program;
    Uniform u_mvp;
    Uniform u_color;
    Uniform u_scale;
    Uniform u_font_size;
    Uniform u_grid_dims;

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

        const char* text_v_src = R"(
#version 330 core
layout(location=0) in vec2 pos;
layout(location=1) in vec2 inst_pos;
layout(location=2) in vec2 inst_glyph;
uniform mat4 u_mvp;
uniform float u_scale;
uniform vec2 u_font_size;
uniform vec2 u_grid_dims;
out vec2 v_uv;
void main() {
    vec2 p = inst_pos + pos * u_font_size * u_scale;
    gl_Position = u_mvp * vec4(p, 0.0, 1.0);
    vec2 uv_pos = vec2(pos.x, 1.0 - pos.y);
    v_uv = (inst_glyph + uv_pos) / u_grid_dims;
}
)";
        const char* text_f_src = R"(
#version 330 core
in vec2 v_uv;
uniform sampler2D u_tex;
uniform vec3 u_color;
out vec4 frag;
void main() {
    float a = texture(u_tex, v_uv).r;
    frag = vec4(u_color, a);
}
)";
        program = GlProgram(text_v_src, text_f_src);
        u_mvp = program.uniform("u_mvp");
        u_color = program.uniform("u_color");
        u_scale = program.uniform("u_scale");
        u_font_size = program.uniform("u_font_size");
        u_grid_dims = program.uniform("u_grid_dims");

        mesh.vertex_count = 6;
        mesh.mode = GL_TRIANGLES;
        mesh.add({.data = QUAD2, .bytes = sizeof(QUAD2), .loc = 0, .comps = 2});
        mesh.add({.data = nullptr, .bytes = 0, .loc = 1, .comps = 2, .divisor = true});
        mesh.add({.data = nullptr, .bytes = 0, .loc = 2, .comps = 2, .divisor = true});
    }

    void draw(const char* text,
              float x,
              float top_y,
              float scale = 1.0f,
              const Vec<3>& color = {1.0f, 1.0f, 1.0f}) {
        std::vector<float> inst_pos;
        std::vector<float> inst_glyph;
        const float advance = font.width * scale;

        for (size_t ti = 0; text[ti] != '\0'; ++ti) {
            const int i = glyph_map[static_cast<unsigned char>(text[ti])];
            if (i < 0) continue;
            const float col = float(uint(i) % glyph_cols);
            const float row = float(uint(i) / glyph_cols);
            const float x0 = x + float(ti) * advance;
            inst_pos.insert(inst_pos.end(), {x0, top_y});
            inst_glyph.insert(inst_glyph.end(), {col, row});
        }

        mesh.instance_count = static_cast<GLsizei>(inst_pos.size() / 2);
        if (mesh.instance_count == 0) return;

        mesh.buffers[1].upload(inst_pos, GL_DYNAMIC_DRAW);
        mesh.buffers[2].upload(inst_glyph, GL_DYNAMIC_DRAW);

        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);

        program.use();
        u_mvp.set(mat_ortho(0.0f, float(vp[2]), 0.0f, float(vp[3])));
        u_color.set(color);
        u_scale.set(scale);
        u_font_size.set(Vec<2>{float(font.width), float(font.height)});
        u_grid_dims.set(Vec<2>{float(glyph_cols), float(glyph_rows)});

        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, tex.id);
        mesh.draw();
        glBindTexture(GL_TEXTURE_2D, 0);

        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
    }
};

////////////////////////////////////////////////////////////////////////////////
// Renderer impl

// Scene constants.
constexpr float FOV_DEGS = 100.0f;
constexpr int CITY_RADIUS = 60;  // cells rendered around the player in each direction
constexpr Vec<3> LIGHT_DIR{0.40f, 0.55f, 0.73f};  // world-space direction toward the sun
constexpr int MINIMAP_CELLS = 20;                 // radar radius, in street cells
constexpr int MINIMAP_SIZE = 300;                 // square side, in pixels

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

// Radar-style minimap overlay.
struct Minimap {
    GlProgram program;
    Uniform u_mvp;
    Uniform u_color;
    Uniform u_translate;
    Uniform u_scale;
    Uniform u_radius;
    Mesh disc;
    Mesh ring;

    GlProgram grid_program;
    Uniform g_mvp;
    Uniform g_color;
    Uniform g_radius;
    Uniform g_band_half;
    Mesh grid;

    Minimap() {
        const char* v_src = R"(
#version 330 core
layout(location=0) in vec2 pos;
uniform mat4 u_mvp;
uniform vec2 u_translate;
uniform float u_scale;
out vec2 v_local;
void main(){
  vec2 p = u_scale * pos + u_translate;
  v_local = p;
  gl_Position = u_mvp * vec4(p, 0.0, 1.0);
}
)";
        const char* grid_v_src = R"(
#version 330 core
layout(location=0) in vec2 pos;
layout(location=1) in vec2 inst_band;    // .x = band center (world-relative, along its normal),
                                         // .y = axis (0=vertical, 1=horizontal)
uniform mat4 u_mvp;
uniform float u_radius;
uniform float u_band_half;
out vec2 v_local;
void main(){
  vec2 p;
  if (inst_band.y < 0.5)
    p = vec2(inst_band.x + pos.x * 2.0 * u_band_half, pos.y * 2.0 * u_radius);
  else
    p = vec2(pos.x * 2.0 * u_radius, inst_band.x + pos.y * 2.0 * u_band_half);
  v_local = p;
  gl_Position = u_mvp * vec4(p, 0.0, 1.0);
}
)";
        const char* f_src = R"(
#version 330 core
in vec2 v_local;
uniform vec3 u_color;
uniform float u_radius;
out vec4 frag;
void main(){
  if (dot(v_local, v_local) > u_radius * u_radius) discard;
  frag = vec4(u_color, 1.0);
}
)";
        program = GlProgram(v_src, f_src);
        u_mvp = program.uniform("u_mvp");
        u_color = program.uniform("u_color");
        u_translate = program.uniform("u_translate");
        u_scale = program.uniform("u_scale");
        u_radius = program.uniform("u_radius");

        grid_program = GlProgram(grid_v_src, f_src);
        g_mvp = grid_program.uniform("u_mvp");
        g_color = grid_program.uniform("u_color");
        g_radius = grid_program.uniform("u_radius");
        g_band_half = grid_program.uniform("u_band_half");

        std::vector<float> disc_verts = {0.0f, 0.0f};
        constexpr int n_seg = 64;  // circle smoothness
        for (int i = 0; i <= n_seg; ++i) {
            float a = 2.0f * M_PIf * float(i % n_seg) / float(n_seg);
            disc_verts.insert(disc_verts.end(), {std::cos(a), std::sin(a)});
        }
        disc.mode = GL_TRIANGLE_FAN;
        disc.vertex_count = static_cast<GLsizei>(disc_verts.size() / 2);
        disc.add({.data = disc_verts.data(),
                  .bytes = static_cast<GLsizeiptr>(disc_verts.size() * sizeof(float)),
                  .loc = 0,
                  .comps = 2});
        ring.mode = GL_LINE_LOOP;
        ring.vertex_count = n_seg;
        // skip first and last vertex (center and duplicate)
        ring.add({.data = disc_verts.data() + 2,
                  .bytes = static_cast<GLsizeiptr>(n_seg * 2 * sizeof(float)),
                  .loc = 0,
                  .comps = 2});
        grid.mode = GL_TRIANGLES;
        grid.vertex_count = 6;
        grid.add({.data = QUAD2C, .bytes = sizeof(QUAD2C), .loc = 0, .comps = 2});
        grid.add({.data = nullptr, .bytes = 0, .loc = 1, .comps = 2, .divisor = true});
    }

    void draw(const GameState& s) {
        const float R = float(City::cell_size) * MINIMAP_CELLS;
        const float px_to_world = 2.0f * R / float(MINIMAP_SIZE);  // 1px radius -> world scale

        const Vec<3>& p = s.drones[0].position;

        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        glViewport(vp[2] - MINIMAP_SIZE - 16, 16, MINIMAP_SIZE, MINIMAP_SIZE);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);

        // Heading-up radar: rotate so the player's forward points up.
        const Vec<3> fwd = rotate(s.drones[0].orientation, Vec<3>{1.0f, 0.0f, 0.0f});
        const float phi = M_PIf / 2.0f - std::atan2(fwd[1], fwd[0]);
        const Quat heading{0.0f, 0.0f, std::sin(phi / 2.0f), std::cos(phi / 2.0f)};
        const Mat4 mvp = mul(mat_ortho(-R, R, -R, R), mat_from(heading, {0.0f, 0.0f, 0.0f}));

        // Background disc.
        program.use();
        u_mvp.set(mvp);
        u_radius.set(R);
        auto draw_disc = [&](float scale, Vec<2> translate, const Vec<3>& color) {
            u_scale.set(scale);
            u_translate.set(translate);
            u_color.set(color);
            disc.draw();
        };
        draw_disc(R, {0.0f, 0.0f}, {0.04f, 0.06f, 0.10f});

        // Street grid: one 2*street_size-wide street per block, at its low edge
        // (ox|oy in [0,street_size)). Street center within a block is at the
        // midpoint of the street cells, i.e. 0.5*street_size*cell from the block
        // origin -- so a drone sitting on an intersection center maps to (0,0).
        grid_program.use();
        {
            const float cell = float(City::cell_size);
            const float block_w = float(City::block_size) * cell;
            const float band_half = 0.5f * float(City::street_size) * cell;
            std::vector<float> bands;  // (center, axis) per band
            const int nx0 = int(std::floor((p[0] - R) / block_w)) - 1;
            const int nx1 = int(std::ceil((p[0] + R) / block_w)) + 1;
            for (int n = nx0; n <= nx1; ++n)
                bands.insert(bands.end(), {n * block_w + band_half - p[0], 0.0f});
            const int ny0 = int(std::floor((p[1] - R) / block_w)) - 1;
            const int ny1 = int(std::ceil((p[1] + R) / block_w)) + 1;
            for (int m = ny0; m <= ny1; ++m)
                bands.insert(bands.end(), {m * block_w + band_half - p[1], 1.0f});
            grid.buffers[1].upload(bands, GL_DYNAMIC_DRAW);

            g_mvp.set(mvp);
            g_radius.set(R);
            g_band_half.set(band_half);
            g_color.set(Vec<3>{0.08f, 0.12f, 0.15f});
            grid.instance_count = static_cast<GLsizei>(bands.size() / 2);
            grid.draw();
        }
        program.use();

        // Concentric radar rings
        u_translate.set(Vec<2>{0.0f, 0.0f});
        u_color.set(Vec<3>{0.1f, 0.4f, 0.5f});
        for (float r : {R, 2.0f * R / 3.0f, R / 3.0f}) {
            u_scale.set(r);
            ring.draw();
        }

        // Drone markers
        for (size_t i = 0; i < s.drones.size(); ++i) {
            const Drone& d = s.drones[i];
            if (d.status != Drone::Status::Alive) continue;
            Vec<3> color = (i == 0) ? Vec<3>{0.2f, 0.75f, 0.95f} : Vec<3>{0.95f, 0.20f, 0.20f};
            draw_disc(4.0f * px_to_world, {d.position[0] - p[0], d.position[1] - p[1]}, color);
        }

        glViewport(vp[0], vp[1], vp[2], vp[3]);
        glEnable(GL_DEPTH_TEST);
    }
};

// Full-screen white-noise static overlay (drawn in clip space; no camera transform).
struct StaticOverlay {
    GlProgram program;
    Uniform u_time;
    Uniform u_pixel_size;
    Mesh quad;

    StaticOverlay() {
        const char* v_src = R"(
#version 330 core
layout(location=0) in vec2 pos;
void main(){ gl_Position = vec4(pos, 0.0, 1.0); }
)";
        const char* f_src = R"(
#version 330 core
uniform float u_time;
uniform float u_pixel_size;
out vec4 frag;
float hash(vec3 p) {
    p = fract(p * 0.1031);
    p += dot(p, p.zyx + 31.32);
    return fract((p.x + p.y) * p.z);
}
void main() {
    vec2 block = floor(gl_FragCoord.xy / u_pixel_size) * u_pixel_size;
    float n = hash(vec3(block, u_time));
    frag = vec4(vec3(n), 1.0);
}
)";
        program = GlProgram(v_src, f_src);
        u_time = program.uniform("u_time");
        u_pixel_size = program.uniform("u_pixel_size");
        constexpr float fs_tri[] = {-1.0f, -1.0f, 3.0f, -1.0f, -1.0f, 3.0f};
        quad.mode = GL_TRIANGLES;
        quad.vertex_count = 3;
        quad.add({.data = fs_tri, .bytes = sizeof(fs_tri), .loc = 0, .comps = 2});
    }

    void draw(uint time, uint pixel_size) const {
        glDisable(GL_DEPTH_TEST);
        program.use();
        u_time.set(time % (1 << 20));
        u_pixel_size.set(pixel_size);
        quad.draw();
        glEnable(GL_DEPTH_TEST);
    }
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

    GlProgram drone_program;
    Uniform u_drone_mvp;
    Uniform u_drone_model;
    Uniform u_drone_color;
    Uniform u_drone_light_dir;
    Uniform u_drone_cam_pos;

    Mesh drone;
    Mesh floor;
    Mesh buildings;

    Minimap minimap;
    TextBatch text;
    Mat4 projection;

    StaticOverlay overlay;
    uint static_time = 0;

    int64 city_anchor_x = INT64_MAX;
    int64 city_anchor_y = INT64_MAX;

    float fps = 0.0f;
    float fps_window = 0.0f;
    int fps_frames = 0;
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
    // Drone mesh
    std::vector<float> drone_verts;
    auto cube = make_cube();
    auto cyl = make_cylinder(16);
    // Deliberately draw slightly bigger, so crashes seem generous
    float rr = Drone::radius * 1.2, hh = Drone::height;
    for (float ang : {M_PIf / 4.0f, -M_PIf / 4.0f}) {
        Quat qz{0, 0, std::sin(ang / 2), std::cos(ang / 2)};
        Mat4 model = mul(mat_rigid(mat_from(qz), Vec<3>{0, 0, 0}),
                         mat_diag(rr * 2, hh * 0.5f, hh * 0.5f, 1.0f));
        append_vertices(drone_verts, cube, model);
    }
    append_vertices(drone_verts, cube, mat_diag(rr * 0.5f, rr * 0.5f, hh * 0.5f, 1.0f));
    float oo = rr / sqrtf(2);
    const std::array<Vec<3>, 4> offsets = {
        {{oo, oo, 0}, {oo, -oo, 0}, {-oo, oo, 0}, {-oo, -oo, 0}}};
    for (const auto& offset : offsets) {
        Mat4 scale = mat_diag(rr * 0.2f, rr * 0.2f, hh, 1.0f);
        Mat4 model = mul(mat_rigid(mat_from(Quat{1, 0, 0, 0}), offset), scale);
        append_vertices(drone_verts, cyl, model);
    }
    state->drone.mode = GL_TRIANGLES;
    state->drone.vertex_count = static_cast<GLsizei>(drone_verts.size() / 3);
    state->drone.add({.data = drone_verts.data(),
                      .bytes = static_cast<GLsizeiptr>(drone_verts.size() * sizeof(float)),
                      .loc = 0,
                      .comps = 3});

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
    state->city_program = GlProgram(city_v_src, PHONG_F_SRC);
    state->u_city_mvp = state->city_program.uniform("u_mvp");

    const char* drone_v_src = R"(
#version 330 core
layout(location=0) in vec3 pos;
uniform mat4 u_mvp;
uniform mat4 u_model;
uniform vec3 u_color;
out vec3 v_world;
out vec3 v_color;
void main(){
  gl_Position = u_mvp * vec4(pos, 1.0);
  v_world = (u_model * vec4(pos, 1.0)).xyz;
  v_color = u_color;
}
)";
    state->drone_program = GlProgram(drone_v_src, PHONG_F_SRC);
    state->u_drone_mvp = state->drone_program.uniform("u_mvp");
    state->u_drone_model = state->drone_program.uniform("u_model");
    state->u_drone_color = state->drone_program.uniform("u_color");
    state->u_drone_light_dir = state->drone_program.uniform("u_light_dir");
    state->u_drone_cam_pos = state->drone_program.uniform("u_cam_pos");
    state->u_city_cell_size = state->city_program.uniform("u_city_cell_size");
    state->u_city_light_dir = state->city_program.uniform("u_light_dir");
    state->u_city_cam_pos = state->city_program.uniform("u_cam_pos");

    state->buildings.mode = GL_TRIANGLES;
    state->buildings.vertex_count = 36;
    state->buildings.instance_count = 0;
    state->buildings.add({.data = CUBE, .bytes = sizeof(CUBE), .loc = 0, .comps = 3});
    state->buildings.add({.data = nullptr, .bytes = 0, .loc = 1, .comps = 3, .divisor = true});
    state->buildings.add({.data = nullptr, .bytes = 0, .loc = 2, .comps = 3, .divisor = true});

    state->floor.mode = GL_TRIANGLES;
    state->floor.vertex_count = 6;
    state->floor.instance_count = 0;
    state->floor.add({.data = QUAD3, .bytes = sizeof(QUAD3), .loc = 0, .comps = 3});
    state->floor.add({.data = nullptr, .bytes = 0, .loc = 1, .comps = 3, .divisor = true});
    state->floor.add({.data = nullptr, .bytes = 0, .loc = 2, .comps = 3, .divisor = true});

    state->projection = mat_perspective(FOV_DEGS * (M_PIf / 180.0f), 16.0f / 9.0f, 0.1f, 1000.0f);

    // Default state
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glDisable(GL_BLEND);
}

Renderer::~Renderer() = default;

void Renderer::draw(const GameState& s, float dt) {
    constexpr float period = 0.5f;
    ++state->fps_frames;
    state->fps_window += dt;
    ++state->static_time;
    if (state->fps_window >= period) {
        state->fps = float(state->fps_frames) / state->fps_window;
        state->fps_window = 0.0f;
        state->fps_frames = 0;
    }

    glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (s.drones.empty()) return;

    // Chase drone 0
    const Drone& d0 = s.drones[0];
    Vec<3> eye = s.camera_pos;
    Mat4 view_mat = rigid_inverse(mul(mat_from(d0.orientation, eye), CAM_ROTATION));

    // City: buildings and floor tiles
    Mat4 mvp = mul(state->projection, view_mat);
    const City& city = s.city;
    const float city_cell_size = city.cell_size;
    const int64 ax = static_cast<int64>(std::floor(d0.position[0] / city_cell_size));
    const int64 ay = static_cast<int64>(std::floor(d0.position[1] / city_cell_size));
    if (ax != state->city_anchor_x || ay != state->city_anchor_y) {
        state->city_anchor_x = ax;
        state->city_anchor_y = ay;
        std::vector<float> floor_world, floor_color;
        std::vector<float> buildings_world, buildings_color;
        for (int64 dy = -CITY_RADIUS; dy <= CITY_RADIUS; ++dy) {
            for (int64 dx = -CITY_RADIUS; dx <= CITY_RADIUS; ++dx) {
                Cell c = city(ax + dx, ay + dy);
                float wx = float(ax + dx) * city_cell_size;
                float wy = float(ay + dy) * city_cell_size;
                if (c.height > 0.0f) {
                    float k = float((c.hash >> 32) & 0xff) / 255.0f;
                    float shade = 0.2f + 0.2f * k;
                    buildings_world.insert(buildings_world.end(), {wx, wy, c.height});
                    buildings_color.insert(buildings_color.end(), {shade, shade, shade});
                } else {
                    floor_world.insert(floor_world.end(), {wx, wy, 0.0f});
                    floor_color.insert(floor_color.end(), {0.15f, 0.15f, 0.15f});
                }
            }
        }
        state->floor.instance_count = static_cast<GLsizei>(floor_world.size() / 3);
        state->floor.buffers[1].upload(floor_world, GL_DYNAMIC_DRAW);
        state->floor.buffers[2].upload(floor_color, GL_DYNAMIC_DRAW);
        state->buildings.instance_count = static_cast<GLsizei>(buildings_world.size() / 3);
        state->buildings.buffers[1].upload(buildings_world, GL_DYNAMIC_DRAW);
        state->buildings.buffers[2].upload(buildings_color, GL_DYNAMIC_DRAW);
    }
    if (state->floor.instance_count > 0) {
        state->city_program.use();
        state->u_city_mvp.set(mvp);
        state->u_city_cell_size.set(city_cell_size);
        state->u_city_light_dir.set(LIGHT_DIR);
        state->u_city_cam_pos.set(eye);
        state->floor.draw();
    }
    if (state->buildings.instance_count > 0) {
        state->city_program.use();
        state->u_city_mvp.set(mvp);
        state->u_city_cell_size.set(city_cell_size);
        state->u_city_light_dir.set(LIGHT_DIR);
        state->u_city_cam_pos.set(eye);
        state->buildings.draw();
    }

    // Draw drones
    state->drone_program.use();
    state->u_drone_light_dir.set(LIGHT_DIR);
    state->u_drone_cam_pos.set(eye);
    for (size_t i = 0; i < s.drones.size(); ++i) {
        const Drone& d = s.drones[i];
        if (d.status != Drone::Status::Alive) continue;
        Vec<3> col = (i == 0) ? Vec<3>{0.3f, 0.3f, 0.8f} : Vec<3>{0.8f, 0.3f, 0.3f};
        Mat4 model = mat_from(d.orientation, d.position);
        Mat4 mvp = mul(state->projection, mul(view_mat, model));
        state->u_drone_mvp.set(mvp);
        state->u_drone_model.set(model);
        state->u_drone_color.set(col);
        state->drone.draw();
    }

    // Draw minimap (radar overlay)
    state->minimap.draw(s);

    // Death overlay
    if (s.drones[0].status != Drone::Status::Alive) {
        constexpr float font_scale = 5.0f;
        constexpr int pixel_size = 4;

        state->overlay.draw(state->static_time, pixel_size);

        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        const char* label =
            (s.drones[0].status == Drone::Status::Detonated) ? "detonation" : "signal lost";
        const float text_w = float(std::strlen(label)) * state->text.font.width * font_scale;
        const float x = float(vp[2]) / 2.0f - text_w / 2.0f;
        const float y = float(vp[3]) / 2.0f;
        state->text.draw(label, x, y, font_scale, {1.0f, 0.2f, 0.2f});
    }

    // Draw FPS counter
    char fps_label[32];
    std::snprintf(fps_label, sizeof(fps_label), "%d FPS", int(state->fps));

    const float scale = 1.5f;
    const float margin = 8.0f;
    GLint vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);
    const float text_w = float(std::strlen(fps_label)) * state->text.font.width * scale;
    const float x = float(vp[2]) - margin - text_w;
    const float top_y = float(vp[3]) - margin - state->text.font.height * scale;

    state->text.draw(fps_label, x, top_y, scale, {1.0f, 0.4f, 0.4f});
}
