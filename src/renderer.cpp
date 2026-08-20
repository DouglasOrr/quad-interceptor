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

constexpr Vec<3> grey(float v) {
    return {v, v, v};
}

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

std::vector<Vec<3>> make_cylinder(uint steps, bool caps) {
    std::vector<Vec<3>> v;
    for (auto i = 0u; i < steps; ++i) {
        float a0 = 2.0f * pi * float(i) / float(steps);
        float a1 = 2.0f * pi * float(i + 1) / float(steps);
        float x0 = 0.5f * std::cos(a0), y0 = 0.5f * std::sin(a0);
        float x1 = 0.5f * std::cos(a1), y1 = 0.5f * std::sin(a1);
        if (caps) {
            v.push_back({0, 0, 0.5f});
            v.push_back({x0, y0, 0.5f});
            v.push_back({x1, y1, 0.5f});
            v.push_back({0, 0, -0.5f});
            v.push_back({x1, y1, -0.5f});
            v.push_back({x0, y0, -0.5f});
        }
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
uniform vec3 u_ambient;
uniform vec3 u_diffuse;
uniform vec3 u_specular;
uniform float u_shininess;
uniform vec3 u_fog_color;
uniform vec2 u_fog_range; // x = fog_start, y = fog_end
void main(){
  vec3 N = normalize(cross(dFdx(v_world), dFdy(v_world)));
  vec3 L = normalize(u_light_dir);
  vec3 to_cam = u_cam_pos - v_world;
  float dist = length(to_cam);
  vec3 V = to_cam / max(dist, 0.0001);
  vec3 H = normalize(L + V);
  float diff = max(dot(N, L), 0.0);
  float spec = diff > 0.0 ? pow(max(dot(N, H), 0.0), u_shininess) : 0.0;
  vec3 lit = v_color * (u_ambient + u_diffuse * diff) + u_specular * spec;
  float fog = clamp((dist - u_fog_range.x) / (u_fog_range.y - u_fog_range.x), 0.0, 1.0);
  frag = vec4(mix(lit, u_fog_color, fog), 1.0);
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
    void set(const Vec<4>& v) const {
        if (loc >= 0) glUniform4fv(loc, 1, v.data);
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
    static constexpr float base_font_width_ratio = 20.0f / 1080.0f;

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

    Vec<2> base_font_size() const {
        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        const float min_dim = std::min(float(vp[2]), float(vp[3]));
        const float w = min_dim * base_font_width_ratio;
        const float h = w * (float(font.height) / float(font.width));
        return {w, h};
    }

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
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
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
        const auto font_size = base_font_size();
        std::vector<float> inst_pos;
        std::vector<float> inst_glyph;
        const float advance = font_size[0] * scale;

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
        u_font_size.set(font_size);
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

// Phong surface shading. The material owns its property values and their
// uniform locations; init() queries them from a program, use() binds them
// before drawing. color is supplied separately (per-instance or u_color).
struct PhongMaterial {
    Vec<3> ambient;
    Vec<3> diffuse;
    Vec<3> specular;
    float shininess;
    Uniform u_ambient{}, u_diffuse{}, u_specular{}, u_shininess{};

    void init(const GlProgram& program) {
        u_ambient = program.uniform("u_ambient");
        u_diffuse = program.uniform("u_diffuse");
        u_specular = program.uniform("u_specular");
        u_shininess = program.uniform("u_shininess");
    }
    void use() const {
        u_ambient.set(ambient);
        u_diffuse.set(diffuse);
        u_specular.set(specular);
        u_shininess.set(shininess);
    }
};

////////////////////////////////////////////////////////////////////////////////
// Renderer impl

// Scene constants.
constexpr float fov_deg = 100.0f;
constexpr int city_radius = 60;  // cells rendered around the player in each direction
constexpr Vec<3> light_dir{0.40f, 0.55f, 0.73f};  // world-space direction toward the sun
constexpr Vec<3> fog_color{0.15f, 0.15f, 0.2f};
constexpr float fog_end = float(city_radius) * City::cell_size;
constexpr float fog_start = fog_end * 0.5f;
constexpr int minimap_cells = 25;  // radar radius, in street cells
constexpr float minimap_size_ratio = 0.25f;
constexpr float attitude_size_ratio = 0.15f;
constexpr float hud_pad_ratio = 0.02f;

// Fixed rotation from drone axes to OpenGL camera axes. Cameras look down
// local -z (forward) with +y up; we want forward = drone +x, up = drone +z,
// right = drone -y. Equivalently a 120-degree rotation about (1,-1,-1)/sqrt3.
// Stored column-major: col0 = right = -drone_y, col1 = up = drone_z,
// col2 = backward (+z_cam) = -drone_x.
constexpr Mat4 cam_rotation = {
    0.0f,  -1.0f, 0.0f, 0.0f,  // col0: right  = drone -y
    0.0f,  0.0f,  1.0f, 0.0f,  // col1: up     = drone +z
    -1.0f, 0.0f,  0.0f, 0.0f,  // col2: back   = drone -x
    0.0f,  0.0f,  0.0f, 1.0f,
};

// Radar-style minimap overlay.
struct MinimapView {
    GlProgram program;
    Uniform u_mvp;
    Uniform u_color;
    Uniform u_alpha;
    Uniform u_translate;
    Uniform u_scale;
    Uniform u_radius;
    Uniform u_inner;
    Mesh disc;
    Mesh ring;

    GlProgram grid_program;
    Uniform g_mvp;
    Uniform g_color;
    Uniform g_alpha;
    Uniform g_radius;
    Uniform g_band_half;
    Mesh grid;

    MinimapView() {
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
uniform float u_inner;
uniform float u_alpha;
out vec4 frag;
void main(){
  if (dot(v_local, v_local) > u_radius * u_radius) discard;
  if (dot(v_local, v_local) < u_inner * u_inner) discard;
  frag = vec4(u_color, u_alpha);
}
)";
        program = GlProgram(v_src, f_src);
        u_mvp = program.uniform("u_mvp");
        u_color = program.uniform("u_color");
        u_alpha = program.uniform("u_alpha");
        u_translate = program.uniform("u_translate");
        u_scale = program.uniform("u_scale");
        u_radius = program.uniform("u_radius");
        u_inner = program.uniform("u_inner");

        grid_program = GlProgram(grid_v_src, f_src);
        g_mvp = grid_program.uniform("u_mvp");
        g_color = grid_program.uniform("u_color");
        g_alpha = grid_program.uniform("u_alpha");
        g_radius = grid_program.uniform("u_radius");
        g_band_half = grid_program.uniform("u_band_half");

        std::vector<float> disc_verts = {0.0f, 0.0f};
        constexpr int n_seg = 64;  // circle smoothness
        for (int i = 0; i <= n_seg; ++i) {
            float a = 2.0f * pi * float(i % n_seg) / float(n_seg);
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
        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        const float min_dim = std::min(float(vp[2]), float(vp[3]));
        const int minimap_size = int(min_dim * minimap_size_ratio);
        const int pad = int(min_dim * hud_pad_ratio);
        const float R = City::cell_size * minimap_cells;

        const auto& player = player_drone(s);
        const Vec<3>& p = player.position;

        glViewport(vp[2] - minimap_size - pad, pad, minimap_size, minimap_size);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);

        // Heading-up radar: rotate so the player's forward points up.
        const Vec<3> fwd = rotate(player.orientation, Vec<3>{1.0f, 0.0f, 0.0f});
        const float phi = pi / 2.0f - std::atan2(fwd[1], fwd[0]);
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
            const float cell = City::cell_size;
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
            g_alpha.set(1.0f);
            grid.instance_count = static_cast<GLsizei>(bands.size() / 2);
            grid.draw();
        }
        program.use();
        u_alpha.set(1.0f);
        u_inner.set(0.0f);

        // Drone markers
        foreach_drone(s, [&](size_t, const Drone& d) {
            Vec<3> color = (d.kind == Drone::Kind::Player) ? Vec<3>{0.2f, 0.75f, 0.95f}
                                                           : Vec<3>{0.95f, 0.20f, 0.20f};
            Vec<2> rel = {d.position[0] - p[0], d.position[1] - p[1]};
            float dist = length(rel);
            Vec<2> marker_pos = (dist > R && dist > 0.0f) ? (R / dist) * rel : rel;
            draw_disc(12.0f, marker_pos, color);  // note: absolute size scales better
        });

        // Objective marker on minimap
        for (auto i = 0u; i < s.mission.objectives.size(); ++i) {
            const Circle& o = s.mission.objectives[i];
            u_scale.set(o.radius);
            u_translate.set(Vec<2>{o.center[0] - p[0], o.center[1] - p[1]});
            Vec<3> color;
            if (s.mission.type != Mission::Race) {
                color = {0.2f, 0.9f, 0.4f};
            } else if (i == s.next_objective) {
                color = {0.2f, 0.9f, 0.4f};
            } else if (i == s.next_objective + 1) {
                color = {0.95f, 0.85f, 0.2f};
            } else {
                color = {0.5f, 0.5f, 0.5f};
            }
            u_color.set(color);
            ring.draw();
        }

        // Lock-on annulus
        if (player.lock_on > 0.0f) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            u_translate.set(Vec<2>{0.0f, 0.0f});
            u_scale.set(R);
            u_radius.set(R);
            u_inner.set(std::max(0.0f, 1.0f - player.lock_on) * R);
            u_alpha.set(0.2f);
            u_color.set(Vec<3>{0.9f, 0.1f, 0.1f});
            disc.draw();
            glDisable(GL_BLEND);
            u_alpha.set(1.0f);
            u_inner.set(0.0f);
        }

        // Concentric radar rings
        u_translate.set(Vec<2>{0.0f, 0.0f});
        u_color.set(Vec<3>{0.1f, 0.4f, 0.5f});
        for (float r : {R, 2.0f * R / 3.0f, R / 3.0f}) {
            u_scale.set(r);
            ring.draw();
        }

        glViewport(vp[0], vp[1], vp[2], vp[3]);
        glEnable(GL_DEPTH_TEST);
    }
};

// Flight HUD view.
struct HudView {
    GlProgram program;
    Uniform u_radius;
    Uniform u_pitch;
    Uniform u_roll;
    Mesh quad;

    HudView() {
        const char* v_src = R"(
#version 330 core
layout(location=0) in vec2 pos;
uniform float u_radius;
out vec2 v_local;
void main(){
  vec2 p = pos * (2.0 * u_radius);
  v_local = p;
  gl_Position = vec4(pos * 2.0, 0.0, 1.0);
}
)";
        const char* f_src = R"(
#version 330 core
in vec2 v_local;
uniform float u_radius;
uniform float u_pitch;
uniform float u_roll;
out vec4 frag;
void main(){
  float dist2 = dot(v_local, v_local);
  if (dist2 > u_radius * u_radius) discard;
  if (dist2 > (u_radius - 1.25) * (u_radius - 1.25)) {
    frag = vec4(0.2, 0.75, 0.95, 1.0);
    return;
  }

  // Center boresight crosshair (+)
  if ((abs(v_local.x) < 10.0 && abs(v_local.y) < 2.0 && abs(v_local.x) > 3.0) ||
      (abs(v_local.y) < 10.0 && abs(v_local.x) < 2.0 && abs(v_local.y) > 3.0)) {
    frag = vec4(0.2, 0.75, 0.95, 1.0);
    return;
  }

  float c = cos(u_roll);
  float s = sin(u_roll);
  vec2 rot_p = vec2(c * v_local.x - s * v_local.y, s * v_local.x + c * v_local.y);

  float pitch_offset = u_pitch * (u_radius / 1.570796);
  float y_rel = rot_p.y - pitch_offset;

  if (abs(y_rel) < 0.75) {
    frag = vec4(0.65, 0.65, 0.65, 1.0);
  } else if (y_rel < 0.0) {
    frag = vec4(0.14, 0.09, 0.06, 1.0);
  } else {
    frag = vec4(0.10, 0.25, 0.40, 0.95);
  }
}
)";
        program = GlProgram(v_src, f_src);
        u_radius = program.uniform("u_radius");
        u_pitch = program.uniform("u_pitch");
        u_roll = program.uniform("u_roll");

        quad.mode = GL_TRIANGLES;
        quad.vertex_count = 6;
        quad.add({.data = QUAD2C, .bytes = sizeof(QUAD2C), .loc = 0, .comps = 2});
    }

    void draw(const GameState& s, TextBatch& text) {
        const Drone& d = player_drone(s);
        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        const float min_dim = std::min(float(vp[2]), float(vp[3]));
        const uint attitude_size = uint(min_dim * attitude_size_ratio);
        const uint pad = uint(min_dim * hud_pad_ratio);

        // Draw the attitude indicator as a circle in the lower-left
        glViewport(pad, pad, attitude_size, attitude_size);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        const auto R_mat = mat_from(d.orientation);
        float pitch = std::asin(std::clamp(R_mat[2, 0], -1.0f, 1.0f));
        float roll = std::atan2(-R_mat[2, 1], R_mat[2, 2]);

        program.use();
        u_radius.set(attitude_size / 2);
        u_pitch.set(pitch);
        u_roll.set(roll);
        quad.draw();

        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
        glViewport(vp[0], vp[1], vp[2], vp[3]);
        glEnable(GL_DEPTH_TEST);

        // Text readouts to the right of the attitude indicator
        char buf[64];
        const auto font_size = text.base_font_size();
        const float text_x = 3 * float(pad) + float(attitude_size);
        const float text_y0 = float(pad);
        const float scale = 1.0f;
        const float line_h = font_size[1] * scale * 1.5f;

        int p_v = int(std::round(pitch * rad_to_deg / 5.0f)) * 5;
        int r_v = int(std::round(roll * rad_to_deg / 5.0f)) * 5;
        std::snprintf(buf, sizeof(buf), "P/R %+3d %+1d", p_v, r_v);
        text.draw(buf, text_x, text_y0 + line_h * 0.0f, scale, {1.0f, 1.0f, 1.0f});

        int alt_v = int(std::round(d.position[2]));
        Vec<3> alt_col = {1.0f, 1.0f, 1.0f};
        if (d.position[2] >= Drone::lock_on_level) {
            alt_col = {1.0f, 0.2f, 0.2f};
        } else if (d.position[2] >= Drone::lock_on_level - 10.0f) {
            alt_col = {1.0f, 0.6f, 0.0f};
        }
        int vs_v = int(std::round(d.velocity[2]));
        std::snprintf(buf, sizeof(buf), "ALT %3d %+1d", alt_v, vs_v);
        text.draw(buf, text_x, text_y0 + line_h * 1.0f, scale, alt_col);

        int spd_v = int(
            std::round(std::sqrt(d.velocity[0] * d.velocity[0] + d.velocity[1] * d.velocity[1])));
        std::snprintf(buf, sizeof(buf), "SPD %3d", spd_v);
        text.draw(buf, text_x, text_y0 + line_h * 2.0f, scale, {1.0f, 1.0f, 1.0f});

        if (s.mission.starting_lives) {
            std::snprintf(buf, sizeof(buf), "LIVES %1d", s.lives);
            Vec<3> lives_col = (s.lives > 1) ? Vec<3>{0.2f, 0.9f, 0.4f} : Vec<3>{1.0f, 0.2f, 0.2f};
            text.draw(buf, text_x, text_y0 + line_h * 3.0f, scale, lives_col);
        }

        const float timer_scale = 1.5f;
        const float timer_y = pad;
        if (s.mission.type == Mission::Race) {
            float sec = float(s.timer.value_or(0)) * Physics::dt;
            std::snprintf(buf, sizeof(buf), "%.1fs", sec);
            const float timer_w = float(std::strlen(buf)) * font_size[0] * timer_scale;
            const float timer_x = float(vp[2]) / 2.0f - timer_w / 2.0f;
            text.draw(buf, timer_x, timer_y, timer_scale, {1.0f, 1.0f, 1.0f});
        } else if (s.mission.type == Mission::Chase) {
            float sec = float(s.timer.value_or(0)) * Physics::dt;
            std::snprintf(buf, sizeof(buf), "%.1fs  CAUGHT %u", sec, s.kill_count);
            const float timer_w = float(std::strlen(buf)) * font_size[0] * timer_scale;
            const float timer_x = float(vp[2]) / 2.0f - timer_w / 2.0f;
            text.draw(buf, timer_x, timer_y, timer_scale, {1.0f, 1.0f, 1.0f});
        }
    }
};

// Full-screen white-noise static overlay (drawn in clip space; no camera transform).
struct NoiseOverlayView {
    GlProgram program;
    Uniform u_time;
    Uniform u_pixel_size;
    Mesh quad;

    NoiseOverlayView() {
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

    void draw(uint time, const GameState& s, TextBatch& text) const {
        glDisable(GL_DEPTH_TEST);
        program.use();
        u_time.set(time % (1 << 20));
        u_pixel_size.set(4.0f);
        quad.draw();
        glEnable(GL_DEPTH_TEST);

        constexpr float font_scale = 4.0f;
        const auto& player = player_drone(s);
        const char* label =
            (player.status == Drone::Status::Detonated) ? "detonation" : "signal lost";
        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        const auto font_size = text.base_font_size();
        const float text_w = float(std::strlen(label)) * font_size[0] * font_scale;
        const float x = float(vp[2]) / 2.0f - text_w / 2.0f;
        const float y = float(vp[3]) / 2.0f;
        text.draw(label, x, y, font_scale, {1.0f, 0.2f, 0.2f});
    }
};

// Frame-rate counter: samples over a window and draws it top-right.
struct FpsView {
    static constexpr float period = 0.5f;
    float fps = 0.0f;
    float window = 0.0f;
    int frames = 0;

    void draw(float dt, TextBatch& text) {
        ++frames;
        window += dt;
        if (window >= period) {
            fps = float(frames) / window;
            window = 0.0f;
            frames = 0;
        }
        char label[32];
        std::snprintf(label, sizeof(label), "%d FPS", int(fps));
        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        const auto font_size = text.base_font_size();
        const float scale = 1.0f;
        const float margin = 8.0f;
        const float text_w = float(std::strlen(label)) * font_size[0] * scale;
        const float x = float(vp[2]) - margin - text_w;
        const float top_y = float(vp[3]) - margin - font_size[1] * scale;
        text.draw(label, x, top_y, scale, {1.0f, 0.4f, 0.4f});
    }
};

// Instanced city geometry (buildings + floor tiles)
struct CityView {
    GlProgram program;
    Uniform u_mvp;
    Uniform u_light_dir;
    Uniform u_cam_pos;
    Uniform u_fog_color;
    Uniform u_fog_range;
    Mesh floor;
    Mesh buildings;
    PhongMaterial floor_mat{.ambient = grey(0.1),
                            .diffuse = grey(0.9),
                            .specular = grey(0.1),
                            .shininess = 5};
    PhongMaterial building_mat{.ambient = grey(0.2),
                               .diffuse = grey(0.7),
                               .specular = grey(0.5),
                               .shininess = 50};
    int64 anchor_x = INT64_MAX;
    int64 anchor_y = INT64_MAX;

    CityView() {
        const char* v_src = R"(
#version 330 core
layout(location=0) in vec3 pos;
layout(location=1) in vec3 inst_pos;
layout(location=2) in vec3 inst_scale;
layout(location=3) in vec3 inst_color;
uniform mat4 u_mvp;
out vec3 v_world;
out vec3 v_color;
void main(){
  vec3 p = inst_pos + inst_scale * (pos + 0.5);
  gl_Position = u_mvp * vec4(p, 1.0);
  v_world = p;
  v_color = inst_color;
}
)";
        program = GlProgram(v_src, PHONG_F_SRC);
        u_mvp = program.uniform("u_mvp");
        u_light_dir = program.uniform("u_light_dir");
        u_cam_pos = program.uniform("u_cam_pos");
        u_fog_color = program.uniform("u_fog_color");
        u_fog_range = program.uniform("u_fog_range");
        floor_mat.init(program);
        building_mat.init(program);

        buildings.mode = GL_TRIANGLES;
        buildings.vertex_count = 36;
        buildings.instance_count = 0;
        buildings.add({.data = CUBE, .bytes = sizeof(CUBE), .loc = 0, .comps = 3});
        buildings.add({.data = nullptr, .bytes = 0, .loc = 1, .comps = 3, .divisor = true});
        buildings.add({.data = nullptr, .bytes = 0, .loc = 2, .comps = 3, .divisor = true});
        buildings.add({.data = nullptr, .bytes = 0, .loc = 3, .comps = 3, .divisor = true});

        floor.mode = GL_TRIANGLES;
        floor.vertex_count = 6;
        floor.instance_count = 0;
        floor.add({.data = QUAD3, .bytes = sizeof(QUAD3), .loc = 0, .comps = 3});
        floor.add({.data = nullptr, .bytes = 0, .loc = 1, .comps = 3, .divisor = true});
        floor.add({.data = nullptr, .bytes = 0, .loc = 2, .comps = 3, .divisor = true});
        floor.add({.data = nullptr, .bytes = 0, .loc = 3, .comps = 3, .divisor = true});
    }

    void draw(const GameState& s, const Mat4& view_projection_mat, const Vec<3>& eye) {
        const City& city = s.city;
        const float cell = city.cell_size;
        const Drone& player = player_drone(s);
        const int64 ax = static_cast<int64>(std::floor(player.position[0] / cell));
        const int64 ay = static_cast<int64>(std::floor(player.position[1] / cell));
        if (ax != anchor_x || ay != anchor_y) {
            anchor_x = ax;
            anchor_y = ay;
            std::vector<float> floor_pos, floor_scale, floor_color;
            std::vector<float> buildings_pos, buildings_scale, buildings_color;
            for (int64 dy = -city_radius; dy <= city_radius; ++dy) {
                for (int64 dx = -city_radius; dx <= city_radius; ++dx) {
                    Cell c = city({ax + dx, ay + dy});
                    float wx = float(ax + dx) * cell;
                    float wy = float(ay + dy) * cell;
                    if (c.height > 0.0f) {
                        float k = float((c.hash >> 32) & 0xff) / 255.0f;
                        float shade = 0.2f + 0.2f * k;
                        buildings_pos.insert(buildings_pos.end(), {wx, wy, 0.0f});
                        buildings_scale.insert(buildings_scale.end(), {cell, cell, c.height});
                        buildings_color.insert(buildings_color.end(), {shade, shade, shade});
                    } else {
                        floor_pos.insert(floor_pos.end(), {wx, wy, 0.0f});
                        floor_scale.insert(floor_scale.end(), {cell, cell, 0.0f});
                        floor_color.insert(floor_color.end(), {0.15f, 0.15f, 0.15f});
                        if (c.skywalk_height > 0.0f) {
                            float k = float((c.hash >> 32) & 0xff) / 255.0f;
                            float shade = 0.25f + 0.2f * k;
                            constexpr float T = City::skywalk_thickness;
                            auto [ox, oy] = City::block_offset({ax + dx, ay + dy});
                            float sx, sy, px, py;
                            if (ox < int64(City::street_size) && oy >= int64(City::street_size)) {
                                sx = cell;
                                sy = T;
                                px = wx;
                                py = wy + 0.5f * (cell - T);
                            } else {
                                sx = T;
                                sy = cell;
                                px = wx + 0.5f * (cell - T);
                                py = wy;
                            }
                            buildings_pos.insert(buildings_pos.end(), {px, py, c.skywalk_height});
                            buildings_scale.insert(buildings_scale.end(), {sx, sy, T});
                            buildings_color.insert(buildings_color.end(),
                                                   {shade * 0.9f, shade * 1.0f, shade * 1.1f});
                        }
                    }
                }
            }
            floor.instance_count = static_cast<GLsizei>(floor_pos.size() / 3);
            floor.buffers[1].upload(floor_pos, GL_DYNAMIC_DRAW);
            floor.buffers[2].upload(floor_scale, GL_DYNAMIC_DRAW);
            floor.buffers[3].upload(floor_color, GL_DYNAMIC_DRAW);
            buildings.instance_count = static_cast<GLsizei>(buildings_pos.size() / 3);
            buildings.buffers[1].upload(buildings_pos, GL_DYNAMIC_DRAW);
            buildings.buffers[2].upload(buildings_scale, GL_DYNAMIC_DRAW);
            buildings.buffers[3].upload(buildings_color, GL_DYNAMIC_DRAW);
        }
        if (floor.instance_count > 0) {
            program.use();
            u_mvp.set(view_projection_mat);
            floor_mat.use();
            u_light_dir.set(light_dir);
            u_cam_pos.set(eye);
            u_fog_color.set(fog_color);
            u_fog_range.set(Vec<2>{fog_start, fog_end});
            floor.draw();
        }
        if (buildings.instance_count > 0) {
            program.use();
            u_mvp.set(view_projection_mat);
            building_mat.use();
            u_light_dir.set(light_dir);
            u_cam_pos.set(eye);
            u_fog_color.set(fog_color);
            u_fog_range.set(Vec<2>{fog_start, fog_end});
            buildings.draw();
        }
    }
};

constexpr Vec<3> PLAYER_COLOR{0.3f, 0.3f, 0.8f};
constexpr Vec<3> ENEMY_COLOR{0.8f, 0.3f, 0.3f};

struct DroneMaterials {
    PhongMaterial player{.ambient = grey(0.15),
                         .diffuse = grey(0.8),
                         .specular = grey(0.2),
                         .shininess = 20};
    PhongMaterial enemy{.ambient = grey(0.65),
                        .diffuse = grey(0.35),
                        .specular = grey(0.1),
                        .shininess = 20};

    void init(const GlProgram& prog) {
        player.init(prog);
        enemy.init(prog);
    }
    const PhongMaterial& get(bool is_enemy) const { return is_enemy ? enemy : player; }
};

// Drone propeller mesh
struct DronePropView {
    static constexpr float prop_radius = 0.2f;

    static std::vector<Vec<3>> make_propeller() {
        const float tip = prop_radius, root_h = 0.10f * prop_radius, tip_h = 0.04f * prop_radius,
                    thick = 0.02f * prop_radius;
        const Vec<3> t0 = {0, -root_h, thick}, t1 = {tip, -tip_h, thick}, t2 = {tip, tip_h, thick},
                     t3 = {0, root_h, thick};
        const Vec<3> b0 = {0, -root_h, -thick}, b1 = {tip, -tip_h, -thick},
                     b2 = {tip, tip_h, -thick}, b3 = {0, root_h, -thick};
        std::vector<Vec<3>> blade = {t0, t1, t2, t0, t2, t3, b0, b3, b2, b0, b2, b1,
                                     b0, t0, t3, b0, t3, b3, t1, b1, b2, t1, b2, t2,
                                     b0, b1, t1, b0, t1, t0, b3, t3, t2, b3, t2, b2};
        std::vector<Vec<3>> v = blade;
        for (auto p : blade)
            v.push_back({-p[0], -p[1], p[2]});
        return v;
    }

    GlProgram program;
    Uniform u_mvp;
    Uniform u_model;
    Uniform u_color;
    Uniform u_light_dir;
    Uniform u_cam_pos;
    Uniform u_fog_color;
    Uniform u_fog_range;
    Mesh mesh;
    DroneMaterials mat;
    struct DroneProp {
        uint id;
        std::array<float, 4> angles{};
    };
    std::vector<DroneProp> props;

    DronePropView() {
        const char* v_src = R"(
#version 330 core
layout(location=0) in vec3 pos;
layout(location=1) in vec3 inst_pos;
layout(location=2) in float inst_angle;
layout(location=3) in float inst_flip;
uniform mat4 u_mvp;
uniform mat4 u_model;
uniform vec3 u_color;
out vec3 v_world;
out vec3 v_color;
void main(){
  float c = cos(inst_angle);
  float s = sin(inst_angle);
  vec3 py = vec3(pos.x, pos.y * inst_flip, pos.z);
  vec3 p = vec3(c * py.x - s * py.y, s * py.x + c * py.y, py.z);
  p += inst_pos;
  vec4 world = u_model * vec4(p, 1.0);
  gl_Position = u_mvp * vec4(p, 1.0);
  v_world = world.xyz;
  v_color = u_color;
}
)";
        program = GlProgram(v_src, PHONG_F_SRC);
        u_mvp = program.uniform("u_mvp");
        u_model = program.uniform("u_model");
        u_color = program.uniform("u_color");
        u_light_dir = program.uniform("u_light_dir");
        u_cam_pos = program.uniform("u_cam_pos");
        u_fog_color = program.uniform("u_fog_color");
        u_fog_range = program.uniform("u_fog_range");
        mat.init(program);

        auto prop_verts = make_propeller();
        mesh.mode = GL_TRIANGLES;
        mesh.vertex_count = static_cast<GLsizei>(prop_verts.size());
        mesh.add({.data = prop_verts.data(),
                  .bytes = static_cast<GLsizeiptr>(prop_verts.size() * sizeof(Vec<3>)),
                  .loc = 0,
                  .comps = 3});
        mesh.add({.data = nullptr, .bytes = 0, .loc = 1, .comps = 3, .divisor = true});
        mesh.add({.data = nullptr, .bytes = 0, .loc = 2, .comps = 1, .divisor = true});
        mesh.add({.data = nullptr, .bytes = 0, .loc = 3, .comps = 1, .divisor = true});
    }

    void draw(const Mat4& mvp,
              const Mat4& model,
              const Vec<3>& col,
              const Vec<3>& eye,
              const Drone& d,
              bool enemy,
              DroneProp& prop,
              float dt) {
        std::vector<float> inst_pos;
        std::vector<float> inst_angle;
        std::vector<float> inst_flip;
        for (int p = 0; p < 4; ++p) {
            Vec<3> offset = Drone::motor_offsets_m[p];
            offset[2] = Drone::height * 0.55f;
            float dir = (p == 0 || p == 3) ? 1.0f : -1.0f;
            prop.angles[p] += dir * 80.0f * (0.3f + d.motor_speed[p]) * dt;
            float angle = prop.angles[p];

            inst_pos.insert(inst_pos.end(), {offset[0], offset[1], offset[2]});
            inst_angle.push_back(angle);
            inst_flip.push_back(dir);
        }
        mesh.buffers[1].upload(inst_pos, GL_DYNAMIC_DRAW);
        mesh.buffers[2].upload(inst_angle, GL_DYNAMIC_DRAW);
        mesh.buffers[3].upload(inst_flip, GL_DYNAMIC_DRAW);
        mesh.instance_count = static_cast<GLsizei>(inst_pos.size() / 3);
        program.use();
        u_mvp.set(mvp);
        u_model.set(model);
        u_color.set(col);
        u_light_dir.set(light_dir);
        u_cam_pos.set(eye);
        u_fog_color.set(fog_color);
        u_fog_range.set(Vec<2>{fog_start, fog_end});
        mat.get(enemy).use();
        mesh.draw();
    }
};

// Drone lateral speed indicator lights (6 unlit pips on inner body cube: 3 left, 3 right)
struct DroneLightsView {
    GlProgram program;
    Uniform u_mvp;
    Uniform u_lat_vel;
    Mesh mesh;

    DroneLightsView() {
        const char* v_src = R"(
#version 330 core
layout(location=0) in vec3 pos;
layout(location=1) in vec3 inst_local_pos;
layout(location=2) in float inst_threshold;
uniform mat4 u_mvp;
uniform float u_lat_vel;
out vec3 v_color;
void main(){
  vec3 p = inst_local_pos + pos * vec3(0.014, 0.014, 0.007);
  gl_Position = u_mvp * vec4(p, 1.0);
  bool lit = (inst_threshold > 0.0 && u_lat_vel >= inst_threshold) ||
             (inst_threshold < 0.0 && u_lat_vel <= inst_threshold);
  v_color = lit ? vec3(1.0, 0.15, 0.15) : vec3(0.25, 0.25, 0.25);
}
)";
        const char* f_src = R"(
#version 330 core
in vec3 v_color;
out vec4 frag;
void main(){
  frag = vec4(v_color, 1.0);
}
)";
        program = GlProgram(v_src, f_src);
        u_mvp = program.uniform("u_mvp");
        u_lat_vel = program.uniform("u_lat_vel");

        auto cube_verts = make_cube();
        mesh.mode = GL_TRIANGLES;
        mesh.vertex_count = static_cast<GLsizei>(cube_verts.size());
        mesh.add({.data = cube_verts.data(),
                  .bytes = static_cast<GLsizeiptr>(cube_verts.size() * sizeof(Vec<3>)),
                  .loc = 0,
                  .comps = 3});

        struct Pip {
            float x, y, z, threshold;
        };
        const Pip pips[6] = {{-0.22f, 0.03f, 0.10f, 4.0f},   {-0.22f, 0.06f, 0.10f, 8.0f},
                             {-0.22f, 0.09f, 0.10f, 12.0f},  {-0.22f, -0.03f, 0.10f, -4.0f},
                             {-0.22f, -0.06f, 0.10f, -8.0f}, {-0.22f, -0.09f, 0.10f, -12.0f}};
        std::vector<float> inst_pos;
        std::vector<float> inst_thresh;
        for (const auto& p : pips) {
            inst_pos.insert(inst_pos.end(), {p.x, p.y, p.z});
            inst_thresh.push_back(p.threshold);
        }

        mesh.add({.data = inst_pos.data(),
                  .bytes = static_cast<GLsizeiptr>(inst_pos.size() * sizeof(float)),
                  .loc = 1,
                  .comps = 3,
                  .divisor = true});
        mesh.add({.data = inst_thresh.data(),
                  .bytes = static_cast<GLsizeiptr>(inst_thresh.size() * sizeof(float)),
                  .loc = 2,
                  .comps = 1,
                  .divisor = true});
        mesh.instance_count = 6;
    }

    void draw(const Mat4& mvp, const Drone& d) {
        auto R = mat_from(d.orientation);
        auto Rt = transpose(R);
        Vec<3> vel_body = mul(Rt, d.velocity);
        program.use();
        u_mvp.set(mvp);
        u_lat_vel.set(vel_body[1]);  // + = left, - = right
        mesh.draw();
    }
};

// Drone static mesh
struct DroneView {
    GlProgram program;
    Uniform u_mvp;
    Uniform u_model;
    Uniform u_color;
    Uniform u_light_dir;
    Uniform u_cam_pos;
    Uniform u_fog_color;
    Uniform u_fog_range;
    Mesh mesh;
    DronePropView prop;
    DroneLightsView lights;
    DroneMaterials mat;
    Mat4 player_scale = mat_diag(1.0f, 1.0f, 1.0f, 1.0f);
    Mat4 enemy_scale = mat_diag(1.5f, 1.5f, 2.5f, 1.0f);

    DroneView() {
        const char* v_src = R"(
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
        program = GlProgram(v_src, PHONG_F_SRC);
        u_mvp = program.uniform("u_mvp");
        u_model = program.uniform("u_model");
        u_color = program.uniform("u_color");
        u_light_dir = program.uniform("u_light_dir");
        u_cam_pos = program.uniform("u_cam_pos");
        u_fog_color = program.uniform("u_fog_color");
        u_fog_range = program.uniform("u_fog_range");
        mat.init(program);

        std::vector<float> verts;
        auto cube = make_cube();
        auto cyl = make_cylinder(16, /*caps*/ true);
        float rr = 0.5f * length(Drone::motor_offsets_m[0] - Drone::motor_offsets_m[3]);
        float hh = Drone::height;
        for (float ang : {pi / 4.0f, -pi / 4.0f}) {
            Quat qz{0, 0, std::sin(ang / 2), std::cos(ang / 2)};
            Mat4 model = mul(mat_rigid(mat_from(qz), Vec<3>{0, 0, 0}),
                             mat_diag(rr * 2, hh * 0.25f, hh * 0.25f, 1.0f));
            append_vertices(verts, cube, model);
        }
        append_vertices(verts, cube, mat_diag(rr * 0.5f, rr * 0.5f, hh * 0.5f, 1.0f));
        for (const auto& offset : Drone::motor_offsets_m) {
            Mat4 scale = mat_diag(hh * 0.6f, hh * 0.6f, hh, 1.0f);
            Mat4 model = mul(mat_rigid(mat_from(Quat{1, 0, 0, 0}), offset), scale);
            append_vertices(verts, cyl, model);
        }
        mesh.mode = GL_TRIANGLES;
        mesh.vertex_count = static_cast<GLsizei>(verts.size() / 3);
        mesh.add({.data = verts.data(),
                  .bytes = static_cast<GLsizeiptr>(verts.size() * sizeof(float)),
                  .loc = 0,
                  .comps = 3});
    }

    void draw(const GameState& s, const Mat4& view_projection_mat, const Vec<3>& eye, float dt) {
        sync_parallel(
            s.drones, prop.props,
            [](const Drone& d) { return DronePropView::DroneProp{.id = d.id}; },
            [&](const Drone& d, DronePropView::DroneProp& p) {
                if (d.status != Drone::Status::Alive) return;
                const bool enemy = (d.kind == Drone::Kind::Enemy);
                const Vec<3>& col = enemy ? ENEMY_COLOR : PLAYER_COLOR;
                Mat4 model = mul(mat_rigid(mat_from(d.orientation), d.position),
                                 enemy ? enemy_scale : player_scale);
                Mat4 mvp = mul(view_projection_mat, model);
                program.use();
                u_light_dir.set(light_dir);
                u_cam_pos.set(eye);
                u_fog_color.set(fog_color);
                u_fog_range.set(Vec<2>{fog_start, fog_end});
                u_mvp.set(mvp);
                u_model.set(model);
                u_color.set(col);
                mat.get(enemy).use();
                mesh.draw();
                prop.draw(mvp, model, col, eye, d, enemy, p, dt);
                lights.draw(mvp, d);
            });
    }
};

// Translucent objective cylinder
struct ObjectiveView {
    GlProgram program;
    Uniform u_mvp;
    Uniform u_color;
    Mesh mesh;

    ObjectiveView() {
        const char* v_src = R"(
#version 330 core
layout(location=0) in vec3 pos;
uniform mat4 u_mvp;
void main(){
  gl_Position = u_mvp * vec4(pos, 1.0);
}
)";
        const char* f_src = R"(
#version 330 core
uniform vec4 u_color;
out vec4 frag;
void main(){
  frag = u_color;
}
)";
        program = GlProgram(v_src, f_src);
        u_mvp = program.uniform("u_mvp");
        u_color = program.uniform("u_color");

        auto cyl_verts = make_cylinder(48, /*caps*/ false);
        mesh.mode = GL_TRIANGLES;
        mesh.vertex_count = static_cast<GLsizei>(cyl_verts.size());
        mesh.add({.data = cyl_verts.data(),
                  .bytes = static_cast<GLsizeiptr>(cyl_verts.size() * sizeof(Vec<3>)),
                  .loc = 0,
                  .comps = 3});
    }

    void draw(const GameState& s, const Mat4& view_projection_mat) {
        constexpr float cyl_height = 200.0f;
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        program.use();
        for (size_t i = 0; i < s.mission.objectives.size(); ++i) {
            const Circle& o = s.mission.objectives[i];
            Vec<4> color;
            if (s.mission.type != Mission::Race) {
                color = {0.2f, 0.85f, 0.4f, 0.25f};
            } else if (i == s.next_objective) {
                color = {0.2f, 0.85f, 0.4f, 0.25f};
            } else if (i == s.next_objective + 1) {
                color = {0.95f, 0.85f, 0.2f, 0.25f};
            } else {
                color = {0.5f, 0.5f, 0.5f, 0.25f};
            }
            u_color.set(color);
            Mat4 model = mat_rigid(mat_from(Quat{0.0f, 0.0f, 0.0f, 1.0f}),
                                   {o.center[0], o.center[1], cyl_height * 0.5f});
            Mat4 scale = mat_diag(o.radius * 2.0f, o.radius * 2.0f, cyl_height, 1.0f);
            Mat4 mvp = mul(view_projection_mat, mul(model, scale));
            u_mvp.set(mvp);
            mesh.draw();
        }
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
};

}  // namespace

struct Renderer::State {
    GlProgram program;
    Uniform u_mvp;
    Uniform u_color;

    CityView city;
    DroneView drone;
    ObjectiveView objective;
    MinimapView minimap;
    HudView hud;
    NoiseOverlayView overlay;
    FpsView fps;
    TextBatch text;
    Mat4 projection;

    uint static_time = 0;
};

Renderer::Renderer() : state(std::make_unique<State>()) {
    state->projection = mat_perspective(fov_deg * deg_to_rad, 16.0f / 9.0f, 0.1f, 1000.0f);

    // Default state
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glDisable(GL_BLEND);
}

Renderer::~Renderer() = default;

void Renderer::draw(const GameState& s, float dt) {
    ++state->static_time;

    glClearColor(fog_color[0], fog_color[1], fog_color[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (s.drones.empty()) return;
    const auto& player = player_drone(s);

    // Camera view
    Mat4 view_mat = rigid_inverse(mul(mat_from(player.orientation, s.camera_pos), cam_rotation));
    Mat4 view_projection_mat = mul(state->projection, view_mat);

    // Scene
    state->city.draw(s, view_projection_mat, s.camera_pos);
    state->drone.draw(s, view_projection_mat, s.camera_pos, dt);
    state->objective.draw(s, view_projection_mat);

    // Overlays
    state->minimap.draw(s);
    state->hud.draw(s, state->text);
    if (player.status != Drone::Status::Alive)
        state->overlay.draw(state->static_time, s, state->text);
    state->fps.draw(dt, state->text);
}
