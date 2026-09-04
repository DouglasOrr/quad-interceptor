#include "game.hpp"
#include "platform.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace {

// Common helper to convert scalar float to grey Vec<3>
constexpr Vec<3> grey(float v) {
    return {v, v, v};
}

////////////////////////////////////////////////////////////////////////////////
// Mesh Primitives Generators

std::vector<Vec<3>> make_cube() {
    std::vector<Vec<3>> v;
    v.reserve(36);
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
    v.reserve(steps * (caps ? 12 : 6));
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

std::vector<float> make_disc(uint segs) {
    std::vector<float> v;
    v.reserve((segs + 1) * 2);
    v.push_back(0.0f);
    v.push_back(0.0f);
    for (uint i = 0; i <= segs; ++i) {
        float a = 2.0f * pi * float(i % segs) / float(segs);
        v.push_back(std::cos(a));
        v.push_back(std::sin(a));
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

// 2D quads and shapes
constexpr float quad2_corner[] = {0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f,
                                  0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f};
constexpr float quad2_centered[] = {-0.5f, -0.5f, 0.5f, -0.5f, 0.5f,  0.5f,
                                    -0.5f, -0.5f, 0.5f, 0.5f,  -0.5f, 0.5f};
constexpr float quad3_centered[] = {-0.5f, -0.5f, 0.0f, 0.5f, -0.5f, 0.0f, 0.5f,  0.5f, 0.0f,
                                    -0.5f, -0.5f, 0.0f, 0.5f, 0.5f,  0.0f, -0.5f, 0.5f, 0.0f};

////////////////////////////////////////////////////////////////////////////////
// OpenGL RAII Wrappers

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
    void set(int i) const {
        if (loc >= 0) glUniform1i(loc, i);
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
            std::fprintf(stderr, "program link error: %s\n", log);
            glDeleteProgram(id);
            id = 0;
            gassert(false);
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
            std::fprintf(stderr, "shader compile error: %s\n--- source ---\n%s\n", log, src);
            glDeleteShader(s);
            gassert(false);
        }
        return s;
    }
};

////////////////////////////////////////////////////////////////////////////////
// Contiguous Instance Layouts & Generic Mesh Helper

struct Instance3D {
    Vec<3> pos;
    Vec<3> scale;
    Vec<3> color;
};

struct CornerInst {
    Vec<4> h;
    Vec<4> c;
};

struct WindowInst {
    Vec<3> pos;
    Vec<2> tangent;
};

struct Mesh {
    GlVertexArray vao;
    GlBuffer vbo;
    GlBuffer instance_vbo;
    GlBuffer corners_vbo;
    GLsizei vertex_count = 0;
    GLsizei instance_count = 0;
    GLenum mode = GL_TRIANGLES;

    void set_vertices(const void* data, GLsizeiptr bytes, int comps = 3, GLenum type = GL_FLOAT) {
        gassert(type == GL_FLOAT);
        vertex_count = static_cast<GLsizei>(bytes / (comps * sizeof(float)));
        vbo.upload(data, bytes, GL_STATIC_DRAW);
        glBindVertexArray(vao.id);
        glBindBuffer(GL_ARRAY_BUFFER, vbo.id);
        glVertexAttribPointer(0, comps, type, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }

    template <typename T>
    void set_vertices(const std::vector<T>& v, int comps = 3) {
        set_vertices(v.data(), static_cast<GLsizeiptr>(v.size() * sizeof(T)), comps);
    }

    void setup_instances_3d() {
        glBindVertexArray(vao.id);
        glBindBuffer(GL_ARRAY_BUFFER, instance_vbo.id);
        constexpr GLsizei stride = sizeof(Instance3D);
        // loc 1: pos
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(Instance3D, pos));
        glEnableVertexAttribArray(1);
        glVertexAttribDivisor(1, 1);
        // loc 2: scale
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(Instance3D, scale));
        glEnableVertexAttribArray(2);
        glVertexAttribDivisor(2, 1);
        // loc 3: color
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(Instance3D, color));
        glEnableVertexAttribArray(3);
        glVertexAttribDivisor(3, 1);
        glBindVertexArray(0);
    }

    void upload_instances(const std::vector<Instance3D>& instances) {
        instance_count = static_cast<GLsizei>(instances.size());
        if (instance_count > 0) {
            instance_vbo.upload(instances, GL_DYNAMIC_DRAW);
        }
    }

    // Optional per-instance corner heights/colours (attributes 4-5), bound only
    // where wanted; without them the attributes read as zero and do nothing.
    void setup_instances_corners() {
        glBindVertexArray(vao.id);
        glBindBuffer(GL_ARRAY_BUFFER, corners_vbo.id);
        constexpr GLsizei stride = sizeof(CornerInst);
        glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, stride, (void*)0);
        glEnableVertexAttribArray(4);
        glVertexAttribDivisor(4, 1);
        glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(CornerInst, c));
        glEnableVertexAttribArray(5);
        glVertexAttribDivisor(5, 1);
        glBindVertexArray(0);
    }

    void upload_corners(const std::vector<CornerInst>& corners) {
        gassert(instance_count == GLsizei(corners.size()));
        if (instance_count > 0) {
            corners_vbo.upload(corners, GL_DYNAMIC_DRAW);
        }
    }

    void setup_instances_windows() {
        glBindVertexArray(vao.id);
        glBindBuffer(GL_ARRAY_BUFFER, instance_vbo.id);
        constexpr GLsizei stride = sizeof(WindowInst);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(WindowInst, pos));
        glEnableVertexAttribArray(1);
        glVertexAttribDivisor(1, 1);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride,
                              (void*)offsetof(WindowInst, tangent));
        glEnableVertexAttribArray(2);
        glVertexAttribDivisor(2, 1);
        glBindVertexArray(0);
    }

    void upload_window_instances(const std::vector<WindowInst>& inst) {
        instance_count = static_cast<GLsizei>(inst.size());
        if (instance_count > 0) {
            instance_vbo.upload(inst, GL_DYNAMIC_DRAW);
        }
    }

    void draw() const {
        if (vertex_count <= 0) return;
        glBindVertexArray(vao.id);
        if (instance_count > 0)
            glDrawArraysInstanced(mode, 0, vertex_count, instance_count);
        else
            glDrawArrays(mode, 0, vertex_count);
        glBindVertexArray(0);
    }
};

////////////////////////////////////////////////////////////////////////////////
// Material Properties

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
// Shaders

// Phong lighting fragment shader, shared by the instanced and model programs
// and reused for instanced windows (see WindowView).
const char* phong_f_src = R"(
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
uniform vec2 u_fog_range;
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

struct PhongShader {
    GlProgram inst_prog;
    GlProgram model_prog;

    // Uniforms
    Uniform u_inst_mvp, u_inst_light_dir, u_inst_cam_pos, u_inst_fog_color, u_inst_fog_range;
    Uniform u_inst_hills;
    Uniform u_model_mvp, u_model_mat, u_model_color, u_model_light_dir, u_model_cam_pos,
        u_model_fog_color, u_model_fog_range;

    PhongShader() { init(); }

    // Build a Phong-lit program from a custom vertex shader, reusing the shared
    // fragment so callers get the same lighting for their own geometry.
    GlProgram phong_program(const char* v_src) const { return GlProgram(v_src, phong_f_src); }

    void init() {
        const char* inst_v_src = R"(
#version 330 core
layout(location=0) in vec3 pos;
layout(location=1) in vec3 inst_pos;
layout(location=2) in vec3 inst_scale;
layout(location=3) in vec3 inst_color;
layout(location=4) in vec4 inst_corner_h;
layout(location=5) in vec4 inst_corner_c;
uniform mat4 u_mvp;
uniform float u_hills;
out vec3 v_world;
out vec3 v_color;
void main(){
  vec3 p = inst_pos + inst_scale * (pos + 0.5);
  vec2 a = pos.xy + 0.5;
  vec4 f = vec4((1.0 - a.x) * (1.0 - a.y), a.x * (1.0 - a.y), a.x * a.y, (1.0 - a.x) * a.y);
  p.z += u_hills * dot(f, inst_corner_h);
  gl_Position = u_mvp * vec4(p, 1.0);
  v_world = p;
  v_color = mix(inst_color, dot(f, inst_corner_c) * inst_color, u_hills);
}
)";
        const char* model_v_src = R"(
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
        inst_prog = GlProgram(inst_v_src, phong_f_src);
        u_inst_mvp = inst_prog.uniform("u_mvp");
        u_inst_light_dir = inst_prog.uniform("u_light_dir");
        u_inst_cam_pos = inst_prog.uniform("u_cam_pos");
        u_inst_fog_color = inst_prog.uniform("u_fog_color");
        u_inst_fog_range = inst_prog.uniform("u_fog_range");
        u_inst_hills = inst_prog.uniform("u_hills");

        model_prog = GlProgram(model_v_src, phong_f_src);
        u_model_mvp = model_prog.uniform("u_mvp");
        u_model_mat = model_prog.uniform("u_model");
        u_model_color = model_prog.uniform("u_color");
        u_model_light_dir = model_prog.uniform("u_light_dir");
        u_model_cam_pos = model_prog.uniform("u_cam_pos");
        u_model_fog_color = model_prog.uniform("u_fog_color");
        u_model_fog_range = model_prog.uniform("u_fog_range");
    }

    void set_scene_instanced(const Mat4& mvp,
                             const Vec<3>& light_dir,
                             const Vec<3>& eye,
                             const Vec<3>& fog_color,
                             Vec<2> fog_range,
                             float hills) const {
        inst_prog.use();
        u_inst_mvp.set(mvp);
        u_inst_light_dir.set(light_dir);
        u_inst_cam_pos.set(eye);
        u_inst_fog_color.set(fog_color);
        u_inst_fog_range.set(fog_range);
        u_inst_hills.set(hills);
    }

    void set_scene_model(const Vec<3>& light_dir,
                         const Vec<3>& eye,
                         const Vec<3>& fog_color,
                         Vec<2> fog_range) const {
        model_prog.use();
        u_model_light_dir.set(light_dir);
        u_model_cam_pos.set(eye);
        u_model_fog_color.set(fog_color);
        u_model_fog_range.set(fog_range);
    }
};

struct FlatShader {
    static constexpr const char* flat_f_src = R"(
#version 330 core
in vec2 v_local;
in vec3 v_color;
uniform vec3 u_color;
uniform float u_alpha;
uniform float u_radius;
uniform float u_inner;
uniform int u_use_instancing;
out vec4 frag;
void main(){
  float d2 = dot(v_local, v_local);
  if (u_radius > 0.0 && d2 > u_radius * u_radius) discard;
  if (u_inner > 0.0 && d2 < u_inner * u_inner) discard;
  vec3 col = (u_use_instancing != 0) ? v_color : u_color;
  frag = vec4(col, u_alpha);
}
)";

    GlProgram prog;

    Uniform u_mvp, u_color, u_alpha, u_translate, u_scale, u_radius, u_inner, u_use_instancing;

    FlatShader() { init(); }

    void init() {
        const char* v_src = R"(
#version 330 core
layout(location=0) in vec3 pos;
layout(location=1) in vec2 inst_pos;
layout(location=2) in float inst_scale;
layout(location=3) in vec3 inst_color;
uniform mat4 u_mvp;
uniform vec2 u_translate;
uniform float u_scale;
uniform int u_use_instancing;
out vec2 v_local;
out vec3 v_color;
void main(){
  vec2 p = (u_use_instancing != 0) ? (inst_scale * pos.xy + inst_pos) : (u_scale * pos.xy + u_translate);
  v_local = p;
  v_color = (u_use_instancing != 0) ? inst_color : vec3(1.0);
  gl_Position = u_mvp * vec4((u_use_instancing == 0 && u_scale == 1.0) ? pos : vec3(p, pos.z), 1.0);
}
)";
        prog = GlProgram(v_src, flat_f_src);
        u_mvp = prog.uniform("u_mvp");
        u_color = prog.uniform("u_color");
        u_alpha = prog.uniform("u_alpha");
        u_translate = prog.uniform("u_translate");
        u_scale = prog.uniform("u_scale");
        u_radius = prog.uniform("u_radius");
        u_inner = prog.uniform("u_inner");
        u_use_instancing = prog.uniform("u_use_instancing");
    }
};

////////////////////////////////////////////////////////////////////////////////
// Scene Constants

constexpr float fov_deg = 100.0f;
constexpr int view_radius = 60;
constexpr Vec<3> light_dir{0.40f, 0.55f, 0.73f};
constexpr Vec<3> fog_color{0.15f, 0.15f, 0.2f};
constexpr float fog_end = float(view_radius) * Terrain::cell_size;
constexpr float fog_start = fog_end * 0.5f;
constexpr Vec<3> city_floor_color{0.15f, 0.15f, 0.15f};
constexpr Vec<3> plains_floor_color{0.10f, 0.16f, 0.08f};
constexpr uint64 tag_tint = 0x54494e54ull;

constexpr uint window_view_radius = 25;
constexpr uint64 tag_win = 0x06227085ull;
constexpr float window_margin = 3.0f;
constexpr int window_cols = 2;
constexpr float window_row_h = 8.0f;

constexpr int minimap_cells = 25;
constexpr float minimap_size_ratio = 0.25f;
constexpr float attitude_size_ratio = 0.15f;
constexpr float hud_pad_ratio = 0.02f;

constexpr Mat4 cam_rotation = {
    0.0f,  -1.0f, 0.0f, 0.0f,  // col0: right  = drone -y
    0.0f,  0.0f,  1.0f, 0.0f,  // col1: up     = drone +z
    -1.0f, 0.0f,  0.0f, 0.0f,  // col2: back   = drone -x
    0.0f,  0.0f,  0.0f, 1.0f,
};

constexpr Vec<3> PLAYER_COLOR{0.3f, 0.3f, 0.8f};
constexpr Vec<3> ENEMY_COLOR{0.8f, 0.3f, 0.3f};

////////////////////////////////////////////////////////////////////////////////
// Subsystems

struct WiresView {
    static constexpr int segs_per_span = 8;

    GlProgram program;
    Uniform u_mvp, u_cam_pos, u_fog_color, u_fog_range;
    Mesh mesh;
    std::vector<float> ends_a;
    std::vector<float> ends_b;

    WiresView() {
        const char* v_src = R"(
#version 330 core
layout(location=0) in float t;
layout(location=1) in vec3 inst_a;
layout(location=2) in vec3 inst_b;
uniform mat4 u_mvp;
out vec3 v_world;
void main(){
  float sag = 0.08 * distance(inst_a, inst_b);
  vec3 mid = vec3(0.5 * (inst_a.x + inst_b.x), 0.5 * (inst_a.y + inst_b.y),
                  min(inst_a.z, inst_b.z) - sag);
  float u = 1.0 - t;
  vec3 p = u * u * inst_a + 2.0 * u * t * mid + t * t * inst_b;
  gl_Position = u_mvp * vec4(p, 1.0);
  v_world = p;
}
)";
        const char* f_src = R"(
#version 330 core
in vec3 v_world;
uniform vec3 u_cam_pos;
uniform vec3 u_fog_color;
uniform vec2 u_fog_range;
out vec4 frag;
void main(){
  float dist = length(u_cam_pos - v_world);
  float fog = clamp((dist - u_fog_range.x) / (u_fog_range.y - u_fog_range.x), 0.0, 1.0);
  frag = vec4(mix(vec3(0.05, 0.05, 0.07), u_fog_color, fog), 1.0);
}
)";
        program = GlProgram(v_src, f_src);
        u_mvp = program.uniform("u_mvp");
        u_cam_pos = program.uniform("u_cam_pos");
        u_fog_color = program.uniform("u_fog_color");
        u_fog_range = program.uniform("u_fog_range");

        std::vector<float> ts;
        ts.reserve(segs_per_span * 2);
        for (int i = 0; i < segs_per_span; ++i) {
            ts.push_back(float(i) / float(segs_per_span));
            ts.push_back(float(i + 1) / float(segs_per_span));
        }
        mesh.mode = GL_LINES;
        mesh.set_vertices(ts, 1);

        glBindVertexArray(mesh.vao.id);
        mesh.instance_vbo.upload((const void*)nullptr, 0);  // allocate
        glBindBuffer(GL_ARRAY_BUFFER, mesh.instance_vbo.id);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribDivisor(1, 1);
        glBindVertexArray(0);
    }

    void span(const Terrain& terr, Terrain::Pos cur, Terrain::Pos prev, bool along_x) {
        auto a = terr(cur);
        auto b = terr(prev);
        auto on_run = [&](const Terrain::PoleData& p) { return along_x ? p.pylon_x : p.pylon_y; };
        if (a.type != Terrain::CellType::Pole || !on_run(a.pole)) return;
        if (b.type != Terrain::CellType::Pole || !on_run(b.pole)) return;
        auto cc = Terrain::cell_center(cur);
        auto cp = Terrain::cell_center(prev);
        Vec<3> A{cp[0], cp[1], Terrain::ground_height(b, {0.5f, 0.5f}) + b.pole.top - 2.0f};
        Vec<3> B{cc[0], cc[1], Terrain::ground_height(a, {0.5f, 0.5f}) + a.pole.top - 2.0f};
        if (length(B - A) < 1.0f) return;
        Vec<2> perp = along_x ? Vec<2>{0.0f, 1.0f} : Vec<2>{1.0f, 0.0f};
        for (float w : {-0.9f, 0.0f, 0.9f}) {
            Vec<3> a2 = A + Vec<3>{perp[0] * w, perp[1] * w, 0.0f};
            Vec<3> b2 = B + Vec<3>{perp[0] * w, perp[1] * w, 0.0f};
            ends_a.insert(ends_a.end(), {a2[0], a2[1], a2[2]});
            ends_b.insert(ends_b.end(), {b2[0], b2[1], b2[2]});
        }
    }

    void upload() {
        mesh.instance_count = static_cast<GLsizei>(ends_a.size() / 3);
        if (mesh.instance_count > 0) {
            std::vector<float> combined = ends_a;
            combined.insert(combined.end(), ends_b.begin(), ends_b.end());
            mesh.instance_vbo.upload(combined, GL_DYNAMIC_DRAW);

            glBindVertexArray(mesh.vao.id);
            glBindBuffer(GL_ARRAY_BUFFER, mesh.instance_vbo.id);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
            glEnableVertexAttribArray(1);
            glVertexAttribDivisor(1, 1);
            GLsizeiptr offset_b = ends_a.size() * sizeof(float);
            glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*)offset_b);
            glEnableVertexAttribArray(2);
            glVertexAttribDivisor(2, 1);
            glBindVertexArray(0);
        }
    }

    void draw(const Mat4& view_projection_mat, const Vec<3>& eye) const {
        if (mesh.instance_count <= 0) return;
        program.use();
        u_mvp.set(view_projection_mat);
        u_cam_pos.set(eye);
        u_fog_color.set(fog_color);
        u_fog_range.set(Vec<2>{fog_start, fog_end});
        mesh.draw();
    }
};

struct TextBatch {
    static constexpr float base_font_width_ratio = 20.0f / 1080.0f;

    GlTexture tex;
    Font font;
    std::array<int, 256> glyph_map{};
    uint glyph_cols = 0;
    uint glyph_rows = 0;
    Mesh mesh;
    GlProgram program;
    Uniform u_mvp, u_color, u_scale, u_font_size, u_grid_dims;
    GlBuffer instance_pos_vbo;
    GlBuffer instance_glyph_vbo;

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

        mesh.set_vertices(quad2_corner, sizeof(quad2_corner), 2);
        glBindVertexArray(mesh.vao.id);
        glBindBuffer(GL_ARRAY_BUFFER, instance_pos_vbo.id);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribDivisor(1, 1);
        glBindBuffer(GL_ARRAY_BUFFER, instance_glyph_vbo.id);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(2);
        glVertexAttribDivisor(2, 1);
        glBindVertexArray(0);
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

        instance_pos_vbo.upload(inst_pos, GL_DYNAMIC_DRAW);
        instance_glyph_vbo.upload(inst_glyph, GL_DYNAMIC_DRAW);

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

struct WindowView {
    GlProgram program;
    Uniform u_mvp, u_light_dir, u_cam_pos, u_fog_color, u_fog_range, u_win_size;
    Mesh dark, lit;
    PhongMaterial dark_mat{.ambient = grey(0.03),
                           .diffuse = Vec<3>{0.02f, 0.03f, 0.05f},
                           .specular = grey(1.0),
                           .shininess = 80};
    PhongMaterial lit_mat{.ambient = Vec<3>{0.9f, 0.7f, 0.45f},
                          .diffuse = Vec<3>{0.1f, 0.2f, 0.1f},
                          .specular = grey(0.6),
                          .shininess = 80};
    std::vector<WindowInst> dark_inst, lit_inst;

    WindowView(const PhongShader& phong) {
        const char* v_src = R"(
#version 330 core
layout(location=0) in vec3 pos;
layout(location=1) in vec3 inst_pos;
layout(location=2) in vec2 inst_tangent;
uniform mat4 u_mvp;
uniform vec2 u_win_size;
out vec3 v_world;
out vec3 v_color;
void main(){
  vec3 p = inst_pos + vec3(inst_tangent * (pos.x * u_win_size.x), pos.y * u_win_size.y);
  gl_Position = u_mvp * vec4(p, 1.0);
  v_world = p;
  v_color = vec3(1.0);
}
)";
        program = phong.phong_program(v_src);
        u_mvp = program.uniform("u_mvp");
        u_light_dir = program.uniform("u_light_dir");
        u_cam_pos = program.uniform("u_cam_pos");
        u_fog_color = program.uniform("u_fog_color");
        u_fog_range = program.uniform("u_fog_range");
        u_win_size = program.uniform("u_win_size");

        dark.set_vertices(quad3_centered, sizeof(quad3_centered));
        dark.setup_instances_windows();
        lit.set_vertices(quad3_centered, sizeof(quad3_centered));
        lit.setup_instances_windows();

        dark_mat.init(program);
        lit_mat.init(program);
    }

    void clear() {
        dark_inst.clear();
        lit_inst.clear();
    }

    void emit(const Terrain& terr, int64 cx, int64 cy, float wx, float wy, float base, float top) {
        const float cell = Terrain::cell_size;
        const float win_h = window_row_h - window_margin;
        if (top - base < win_h + 2.0f * window_margin) return;

        const uint64 hb = hash(terr.seed, tag_win, cx, cy);
        const float lit_rate = hash_u(hb, 0) * hash_u(hb, 0);
        const float span = cell - 2.0f * window_margin;
        const float win_w = (span - float(window_cols - 1) * window_margin) / float(window_cols);
        const float half_w = 0.5f * win_w, half_h = 0.5f * win_h;
        uint idx = 0;

        auto face = [&](float plane, bool along_x, float sign) {
            const Vec<2> tangent = along_x ? Vec<2>{0.0f, 1.0f} : Vec<2>{1.0f, 0.0f};
            for (int i = 0; i < window_cols; ++i) {
                const float cu = window_margin + float(i) * (win_w + window_margin) + half_w;
                for (int r = 0;; ++r) {
                    const float zc = base + window_margin + (float(r) + 0.5f) * window_row_h;
                    if (zc + half_h > top - window_margin) break;
                    const float along = along_x ? wy + cu : wx + cu;
                    const float normal = plane + sign * 0.2f;
                    Vec<3> center = along_x ? Vec<3>{normal, along, zc} : Vec<3>{along, normal, zc};
                    const uint64 h = hash(terr.seed, tag_win, cx, cy, idx++);
                    auto& out = hash_u(h, 16) < lit_rate ? lit_inst : dark_inst;
                    out.push_back({center, tangent});
                }
            }
        };
        face(wx, true, -1.0f);
        face(wx + cell, true, +1.0f);
        face(wy, false, -1.0f);
        face(wy + cell, false, +1.0f);
    }

    void upload() {
        dark.upload_window_instances(dark_inst);
        lit.upload_window_instances(lit_inst);
    }

    void draw(const Mat4& view_projection_mat, const Vec<3>& eye) const {
        if (dark.instance_count <= 0 && lit.instance_count <= 0) return;
        program.use();
        u_mvp.set(view_projection_mat);
        u_light_dir.set(light_dir);
        u_cam_pos.set(eye);
        u_fog_color.set(fog_color);
        u_fog_range.set(Vec<2>{fog_start, fog_end});
        u_win_size.set(Vec<2>{
            (Terrain::cell_size - 2.0f * window_margin - float(window_cols - 1) * window_margin) /
                float(window_cols),
            window_row_h - window_margin});
        if (dark.instance_count > 0) {
            dark_mat.use();
            dark.draw();
        }
        if (lit.instance_count > 0) {
            lit_mat.use();
            lit.draw();
        }
    }
};

struct TerrainView {
    Mesh floor;
    Mesh buildings;
    Mesh pillars;
    WiresView wires;
    WindowView windows;

    PhongMaterial floor_mat{.ambient = grey(0.1),
                            .diffuse = grey(0.9),
                            .specular = grey(0.1),
                            .shininess = 5};
    PhongMaterial building_mat{.ambient = grey(0.2),
                               .diffuse = grey(0.7),
                               .specular = grey(0.2),
                               .shininess = 20};

    int64 anchor_x = INT64_MAX;
    int64 anchor_y = INT64_MAX;

    TerrainView(const PhongShader& phong) : windows(phong) {
        floor.set_vertices(quad3_centered, sizeof(quad3_centered));
        floor.setup_instances_3d();
        floor.setup_instances_corners();

        auto cube_verts = make_cube();
        buildings.set_vertices(cube_verts);
        buildings.setup_instances_3d();

        auto cyl_verts = make_cylinder(20, /*caps*/ true);
        pillars.set_vertices(cyl_verts);
        pillars.setup_instances_3d();

        floor_mat.init(phong.inst_prog);
        building_mat.init(phong.inst_prog);
    }

    void draw(const GameState& s,
              const PhongShader& phong,
              const Mat4& view_projection_mat,
              const Vec<3>& eye) {
        const Terrain& terr = s.terrain;
        const TerrainType& tt = terr.type;
        const float cell = Terrain::cell_size;
        const bool plains = tt.kind == TerrainType::Kind::Plains;
        const Drone& player = player_drone(s);
        const int64 ax = static_cast<int64>(std::floor(player.position[0] / cell));
        const int64 ay = static_cast<int64>(std::floor(player.position[1] / cell));

        if (ax != anchor_x || ay != anchor_y) {
            anchor_x = ax;
            anchor_y = ay;

            std::vector<Instance3D> floor_inst;
            std::vector<CornerInst> floor_corners;
            std::vector<Instance3D> building_inst;
            std::vector<Instance3D> pillar_inst;
            wires.ends_a.clear();
            wires.ends_b.clear();
            windows.clear();

            for (int64 dy = -view_radius; dy <= view_radius; ++dy) {
                for (int64 dx = -view_radius; dx <= view_radius; ++dx) {
                    auto cx = ax + dx, cy = ay + dy;
                    float wx = float(cx) * cell, wy = float(cy) * cell;
                    auto c = terr({cx, cy});

                    if (c.type != Terrain::CellType::Building) {  // has a floor
                        const Vec<3>& g = plains ? plains_floor_color : city_floor_color;
                        CornerInst corners;
                        if (plains) {
                            corners.h = {c.ground[0], c.ground[1], c.ground[3], c.ground[2]};
                            auto tint = [&](int64 x, int64 y) {
                                return hash_u(hash(terr.seed, tag_tint, x, y), 0, 0.72f, 1.05f);
                            };
                            corners.c = {tint(cx, cy), tint(cx + 1, cy), tint(cx + 1, cy + 1),
                                         tint(cx, cy + 1)};
                        } else {
                            corners.h = {0.0f, 0.0f, 0.0f, 0.0f};
                            corners.c = {1.0f, 1.0f, 1.0f, 1.0f};
                        }
                        floor_inst.push_back({{wx, wy, 0.0f}, {cell, cell, 0.0f}, g});
                        floor_corners.push_back(corners);
                    }

                    switch (c.type) {
                        case Terrain::CellType::Building: {
                            const auto& b = c.building;
                            float shade = hash_u(c.hash, 32, 0.2f, 0.4f);
                            building_inst.push_back({{wx, wy, b.base},
                                                     {cell, cell, b.top - b.base},
                                                     {shade, shade, shade}});
                            if (b.podium > 0.0f) {
                                building_inst.push_back({{wx, wy, 0.0f},
                                                         {cell, cell, b.podium},
                                                         {shade, shade, shade}});
                                const float pr = TerrainType::City::pillar_radius;
                                pillar_inst.push_back(
                                    {{wx + 0.5f * cell - pr, wy + 0.5f * cell - pr, b.podium},
                                     {2.0f * pr, 2.0f * pr, b.base - b.podium},
                                     {0.35f, 0.35f, 0.4f}});
                            }
                            if (uint(std::max(std::abs(dx), std::abs(dy))) <= window_view_radius) {
                                windows.emit(terr, cx, cy, wx, wy, b.base, b.top);
                                if (b.podium > 0.0f)
                                    windows.emit(terr, cx, cy, wx, wy, 0.0f, b.podium);
                            }
                            break;
                        }
                        case Terrain::CellType::Skywalk: {
                            float shade = hash_u(c.hash, 32, 0.25f, 0.45f);
                            const float T = TerrainType::City::skywalk_thickness;
                            float sx = c.skywalk.axis_x ? cell : T;
                            float sy = c.skywalk.axis_x ? T : cell;
                            float px = c.skywalk.axis_x ? wx : wx + 0.5f * (cell - T);
                            float py = c.skywalk.axis_x ? wy + 0.5f * (cell - T) : wy;
                            building_inst.push_back({{px, py, c.skywalk.bottom},
                                                     {sx, sy, T},
                                                     {shade * 0.9f, shade * 1.0f, shade * 1.1f}});
                            break;
                        }
                        case Terrain::CellType::Pole: {
                            const float r = c.pole.radius;
                            Vec<3> pole_col = (c.pole.pylon_x || c.pole.pylon_y)
                                                  ? Vec<3>{0.32f, 0.34f, 0.40f}
                                                  : Vec<3>{0.48f, 0.42f, 0.34f};
                            const float base =
                                0.25f * (c.ground[0] + c.ground[1] + c.ground[2] + c.ground[3]);
                            pillar_inst.push_back(
                                {{wx + 0.5f * cell - r, wy + 0.5f * cell - r, base},
                                 {2.0f * r, 2.0f * r, c.pole.top},
                                 pole_col});

                            if (plains && (c.pole.pylon_x || c.pole.pylon_y)) {
                                Terrain::Pos here = {ax + dx, ay + dy};
                                const int64 stride = int64(tt.plains.pylon_stride);
                                if (c.pole.pylon_x)
                                    wires.span(terr, here, {here[0] - stride, here[1]}, true);
                                if (c.pole.pylon_y)
                                    wires.span(terr, here, {here[0], here[1] - stride}, false);
                            }
                            break;
                        }
                        case Terrain::CellType::Empty:
                            break;
                    }
                }
            }
            floor.upload_instances(floor_inst);
            floor.upload_corners(floor_corners);
            buildings.upload_instances(building_inst);
            pillars.upload_instances(pillar_inst);
            windows.upload();
            wires.upload();
        }

        phong.set_scene_instanced(view_projection_mat, light_dir, eye, fog_color,
                                  Vec<2>{fog_start, fog_end}, /*hills*/ 1.0f);

        if (floor.instance_count > 0) {
            floor_mat.use();
            floor.draw();
        }
        phong.set_scene_instanced(view_projection_mat, light_dir, eye, fog_color,
                                  Vec<2>{fog_start, fog_end}, /*hills*/ 0.0f);
        if (buildings.instance_count > 0) {
            building_mat.use();
            buildings.draw();
        }
        if (pillars.instance_count > 0) {
            building_mat.use();
            pillars.draw();
        }
        windows.draw(view_projection_mat, eye);
        wires.draw(view_projection_mat, eye);
    }
};

struct DroneView {
    static constexpr float motor_height_ratio = 0.55f;

    Mesh frame_mesh;
    Mesh prop_mesh;
    Mesh cube_mesh;

    PhongMaterial player_mat{.ambient = grey(0.15),
                             .diffuse = grey(0.8),
                             .specular = grey(0.2),
                             .shininess = 20};
    PhongMaterial enemy_mat{.ambient = grey(0.65),
                            .diffuse = grey(0.35),
                            .specular = grey(0.1),
                            .shininess = 20};

    Mat4 player_scale = mat_diag(1.0f, 1.0f, 1.0f, 1.0f);
    Mat4 enemy_scale = mat_diag(1.5f, 1.5f, 2.5f, 1.0f);

    struct DroneProp {
        uint id;
        std::array<float, 4> angles{};
    };
    std::vector<DroneProp> props;
    std::array<Vec<3>, 4> ref_motor_offsets_m{};
    float ref_width = 0.0f;

    DroneView(const PhongShader& phong) {
        player_mat.init(phong.model_prog);
        enemy_mat.init(phong.model_prog);

        std::vector<float> verts;
        auto cube = make_cube();
        auto cyl = make_cylinder(16, /*caps*/ true);
        const DroneType ref = DroneType::all().front();
        float rr = 0.5f * length(ref.motor_offsets_m[0] - ref.motor_offsets_m[3]);
        float hh = ref.height;

        for (float ang : {pi / 4.0f, -pi / 4.0f}) {
            Quat qz{0, 0, std::sin(ang / 2), std::cos(ang / 2)};
            Mat4 model = mul(mat_rigid(mat_from(qz), Vec<3>{0, 0, 0}),
                             mat_diag(rr * 2, hh * 0.25f, hh * 0.25f, 1.0f));
            append_vertices(verts, cube, model);
        }
        append_vertices(verts, cube, mat_diag(rr * 0.5f, rr * 0.5f, hh * 0.5f, 1.0f));
        for (const auto& offset : ref.motor_offsets_m) {
            Mat4 scale = mat_diag(hh * 0.6f, hh * 0.6f, hh, 1.0f);
            Mat4 model = mul(mat_rigid(mat_from(Quat{1, 0, 0, 0}), offset), scale);
            append_vertices(verts, cyl, model);
        }
        frame_mesh.set_vertices(verts.data(),
                                static_cast<GLsizeiptr>(verts.size() * sizeof(float)));

        // Propeller mesh
        const float tip = 0.2f, root_h = 0.02f, tip_h = 0.008f, thick = 0.004f;
        const Vec<3> t0 = {0, -root_h, thick}, t1 = {tip, -tip_h, thick}, t2 = {tip, tip_h, thick},
                     t3 = {0, root_h, thick};
        const Vec<3> b0 = {0, -root_h, -thick}, b1 = {tip, -tip_h, -thick},
                     b2 = {tip, tip_h, -thick}, b3 = {0, root_h, -thick};
        std::vector<Vec<3>> blade = {t0, t1, t2, t0, t2, t3, b0, b3, b2, b0, b2, b1,
                                     b0, t0, t3, b0, t3, b3, t1, b1, b2, t1, b2, t2,
                                     b0, b1, t1, b0, t1, t0, b3, t3, t2, b3, t2, b2};
        std::vector<Vec<3>> prop_v = blade;
        for (auto p : blade)
            prop_v.push_back({-p[0], -p[1], p[2]});
        prop_mesh.set_vertices(prop_v);

        cube_mesh.set_vertices(cube);

        ref_motor_offsets_m = ref.motor_offsets_m;
        for (auto& o : ref_motor_offsets_m)
            o[2] = ref.height * motor_height_ratio;
        ref_width = 2.0f * (rr + tip);  // motor span (2*rr) plus both blade tips
    }

    void draw_flat(const FlatShader& flat,
                   const Mat4& view,
                   const Mat4& model,
                   const Vec<3>& color) const {
        flat.prog.use();
        flat.u_use_instancing.set(0);
        flat.u_radius.set(0.0f);
        flat.u_inner.set(0.0f);
        flat.u_scale.set(1.0f);
        flat.u_translate.set(Vec<2>{0.0f, 0.0f});
        flat.u_alpha.set(1.0f);
        flat.u_color.set(color);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        flat.u_mvp.set(mul(view, model));
        frame_mesh.draw();
        const Quat id{0.0f, 0.0f, 0.0f, 1.0f};
        for (const auto& off : ref_motor_offsets_m) {
            Mat4 prop_model = mul(model, mat_rigid(mat_from(id), off));
            flat.u_mvp.set(mul(view, prop_model));
            prop_mesh.draw();
        }
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
    }

    void draw(const GameState& s,
              const PhongShader& phong,
              const FlatShader& flat,
              const Mat4& view_projection_mat,
              const Vec<3>& eye,
              float dt) {
        phong.set_scene_model(light_dir, eye, fog_color, Vec<2>{fog_start, fog_end});

        sync_parallel(
            s.drones, props, [](const Drone& d) { return DroneProp{.id = d.id}; },
            [&](const Drone& d, DroneProp& p) {
                if (d.status != Drone::Status::Alive) return;
                const bool enemy = (d.kind == Drone::Kind::Enemy);
                const Vec<3>& col = enemy ? ENEMY_COLOR : PLAYER_COLOR;
                Mat4 model = mul(mat_rigid(mat_from(d.orientation), d.position),
                                 enemy ? enemy_scale : player_scale);
                Mat4 mvp = mul(view_projection_mat, model);

                // Main frame
                phong.model_prog.use();
                phong.u_model_mvp.set(mvp);
                phong.u_model_mat.set(model);
                phong.u_model_color.set(col);
                if (enemy)
                    enemy_mat.use();
                else
                    player_mat.use();
                frame_mesh.draw();

                // Propellers
                for (int m = 0; m < 4; ++m) {
                    Vec<3> offset = d.type.motor_offsets_m[m];
                    offset[2] = d.type.height * motor_height_ratio;
                    float dir = (m == 0 || m == 3) ? 1.0f : -1.0f;
                    p.angles[m] += dir * 80.0f * (0.3f + d.motor_speed[m]) * dt;
                    Quat q_rot = quat_from_axis_angle({0, 0, 1}, p.angles[m]);
                    Mat4 p_model = mul(model, mat_rigid(mat_from(q_rot), offset));
                    Mat4 p_mvp = mul(view_projection_mat, p_model);
                    phong.u_model_mvp.set(p_mvp);
                    phong.u_model_mat.set(p_model);
                    prop_mesh.draw();
                }

                // Speed indicator lights on player drone
                if (!enemy) {
                    auto R = mat_from(d.orientation);
                    Vec<3> vel_body = mul(transpose(R), d.velocity);
                    float lat_vel = vel_body[1];
                    struct Pip {
                        float x, y, z, thresh;
                    };
                    const Pip pips[6] = {
                        {-0.22f, 0.03f, 0.10f, 4.0f},   {-0.22f, 0.06f, 0.10f, 8.0f},
                        {-0.22f, 0.09f, 0.10f, 12.0f},  {-0.22f, -0.03f, 0.10f, -4.0f},
                        {-0.22f, -0.06f, 0.10f, -8.0f}, {-0.22f, -0.09f, 0.10f, -12.0f}};
                    flat.prog.use();
                    flat.u_use_instancing.set(0);
                    flat.u_radius.set(0.0f);
                    flat.u_inner.set(0.0f);
                    flat.u_alpha.set(1.0f);
                    for (const auto& pip : pips) {
                        bool lit = (pip.thresh > 0.0f && lat_vel >= pip.thresh) ||
                                   (pip.thresh < 0.0f && lat_vel <= pip.thresh);
                        Vec<3> pip_col =
                            lit ? Vec<3>{1.0f, 0.15f, 0.15f} : Vec<3>{0.25f, 0.25f, 0.25f};
                        Mat4 pip_local =
                            mul(mat_rigid(mat_diag(1.0f, 1.0f, 1.0f), {pip.x, pip.y, pip.z}),
                                mat_diag(0.014f, 0.014f, 0.007f, 1.0f));
                        Mat4 pip_mvp =
                            mul(view_projection_mat,
                                mul(mat_rigid(mat_from(d.orientation), d.position), pip_local));
                        flat.u_mvp.set(pip_mvp);
                        flat.u_color.set(pip_col);
                        flat.u_scale.set(1.0f);
                        flat.u_translate.set(Vec<2>{0, 0});
                        cube_mesh.draw();
                    }
                }
            });
    }
};

struct MinimapView {
    Mesh disc;
    Mesh ring;
    Mesh grid_quad;
    Mesh markers;
    GlProgram grid_prog;
    Uniform g_mvp, g_color, g_alpha, g_radius, g_band_half;

    MinimapView() {
        const char* grid_v_src = R"(
#version 330 core
layout(location=0) in vec2 pos;
layout(location=1) in vec2 inst_band;
uniform mat4 u_mvp;
uniform float u_radius;
uniform float u_band_half;
out vec2 v_local;
void main(){
  vec2 p = (inst_band.y < 0.5)
      ? vec2(inst_band.x + pos.x * 2.0 * u_band_half, pos.y * 2.0 * u_radius)
      : vec2(pos.x * 2.0 * u_radius, inst_band.x + pos.y * 2.0 * u_band_half);
  v_local = p;
  gl_Position = u_mvp * vec4(p, 0.0, 1.0);
}
)";
        grid_prog = GlProgram(grid_v_src, FlatShader::flat_f_src);
        g_mvp = grid_prog.uniform("u_mvp");
        g_color = grid_prog.uniform("u_color");
        g_alpha = grid_prog.uniform("u_alpha");
        g_radius = grid_prog.uniform("u_radius");
        g_band_half = grid_prog.uniform("u_band_half");

        constexpr int n_seg = 64;
        auto disc_verts = make_disc(uint(n_seg));
        disc.mode = GL_TRIANGLE_FAN;
        disc.set_vertices(disc_verts, 2);

        markers.mode = GL_TRIANGLE_FAN;
        markers.set_vertices(disc_verts, 2);
        markers.setup_instances_3d();

        ring.mode = GL_LINE_LOOP;
        // skip first and last vertex (center and duplicate)
        ring.set_vertices(disc_verts.data() + 2, static_cast<GLsizeiptr>(n_seg * 2 * sizeof(float)),
                          2);

        grid_quad.set_vertices(quad2_centered, sizeof(quad2_centered), 2);
        glBindVertexArray(grid_quad.vao.id);
        glBindBuffer(GL_ARRAY_BUFFER, grid_quad.instance_vbo.id);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribDivisor(1, 1);
        glBindVertexArray(0);
    }

    void draw(const GameState& s, const FlatShader& flat) {
        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        const float min_dim = std::min(float(vp[2]), float(vp[3]));
        const int minimap_size = int(min_dim * minimap_size_ratio);
        const int pad = int(min_dim * hud_pad_ratio);
        const float R = Terrain::cell_size * minimap_cells;

        const auto& player = player_drone(s);
        const Vec<3>& p = player.position;

        glViewport(vp[2] - minimap_size - pad, pad, minimap_size, minimap_size);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);

        const Vec<3> fwd = rotate(player.orientation, Vec<3>{1.0f, 0.0f, 0.0f});
        const float phi = pi / 2.0f - std::atan2(fwd[1], fwd[0]);
        const Quat heading{0.0f, 0.0f, std::sin(phi / 2.0f), std::cos(phi / 2.0f)};
        const Mat4 mvp = mul(mat_ortho(-R, R, -R, R), mat_from(heading, {0.0f, 0.0f, 0.0f}));

        // Background
        flat.prog.use();
        flat.u_use_instancing.set(0);
        flat.u_mvp.set(mvp);
        flat.u_radius.set(R);
        flat.u_inner.set(0.0f);
        flat.u_alpha.set(1.0f);
        flat.u_scale.set(R);
        flat.u_translate.set(Vec<2>{0.0f, 0.0f});
        flat.u_color.set(Vec<3>{0.04f, 0.06f, 0.10f});
        disc.draw();

        // Grid
        if (s.terrain.type.kind == TerrainType::Kind::City) {
            grid_prog.use();
            const float cell = Terrain::cell_size;
            const float block_w = float(s.terrain.type.city.block_size) * cell;
            const float band_half = 0.5f * float(s.terrain.type.city.street_size) * cell;
            std::vector<float> bands;
            const int nx0 = int(std::floor((p[0] - R) / block_w)) - 1;
            const int nx1 = int(std::ceil((p[0] + R) / block_w)) + 1;
            for (int n = nx0; n <= nx1; ++n)
                bands.insert(bands.end(), {n * block_w + band_half - p[0], 0.0f});
            const int ny0 = int(std::floor((p[1] - R) / block_w)) - 1;
            const int ny1 = int(std::ceil((p[1] + R) / block_w)) + 1;
            for (int m = ny0; m <= ny1; ++m)
                bands.insert(bands.end(), {m * block_w + band_half - p[1], 1.0f});

            grid_quad.instance_vbo.upload(bands, GL_DYNAMIC_DRAW);
            g_mvp.set(mvp);
            g_radius.set(R);
            g_band_half.set(band_half);
            g_color.set(Vec<3>{0.08f, 0.12f, 0.15f});
            g_alpha.set(1.0f);
            grid_quad.instance_count = static_cast<GLsizei>(bands.size() / 2);
            grid_quad.draw();
        } else {
            grid_prog.use();
            const float cell = Terrain::cell_size;
            const float lane_period = float(s.terrain.type.plains.pylon_row_period) * cell;
            const float lane_half = 0.5f * cell;
            std::vector<float> bands;
            const int nx0 = int(std::floor((p[0] - R) / lane_period)) - 1;
            const int nx1 = int(std::ceil((p[0] + R) / lane_period)) + 1;
            for (int n = nx0; n <= nx1; ++n)
                bands.insert(bands.end(), {n * lane_period + lane_half - p[0], 0.0f});
            const int ny0 = int(std::floor((p[1] - R) / lane_period)) - 1;
            const int ny1 = int(std::ceil((p[1] + R) / lane_period)) + 1;
            for (int m = ny0; m <= ny1; ++m)
                bands.insert(bands.end(), {m * lane_period + lane_half - p[1], 1.0f});

            grid_quad.instance_vbo.upload(bands, GL_DYNAMIC_DRAW);
            g_mvp.set(mvp);
            g_radius.set(R);
            g_band_half.set(lane_half);
            g_color.set(Vec<3>{0.07f, 0.11f, 0.08f});
            g_alpha.set(1.0f);
            grid_quad.instance_count = static_cast<GLsizei>(bands.size() / 2);
            grid_quad.draw();

            // Footprint dots
            std::vector<Instance3D> mk_inst;
            const int64 cx0 = int64(std::floor((p[0] - R) / Terrain::cell_size));
            const int64 cx1 = int64(std::floor((p[0] + R) / Terrain::cell_size));
            const int64 cy0 = int64(std::floor((p[1] - R) / Terrain::cell_size));
            const int64 cy1 = int64(std::floor((p[1] + R) / Terrain::cell_size));
            for (int64 cy = cy0; cy <= cy1; ++cy) {
                for (int64 cx = cx0; cx <= cx1; ++cx) {
                    auto c = s.terrain({cx, cy});
                    if (c.type != Terrain::CellType::Pole) continue;
                    auto center = Terrain::cell_center({cx, cy});
                    bool pylon = c.pole.pylon_x || c.pole.pylon_y;
                    Vec<3> color = pylon ? Vec<3>{0.45f, 0.5f, 0.55f} : Vec<3>{0.35f, 0.4f, 0.28f};
                    mk_inst.push_back({{center[0] - p[0], center[1] - p[1], 0.0f},
                                       {pylon ? 5.0f : 2.0f, 0.0f, 0.0f},
                                       color});
                }
            }
            markers.upload_instances(mk_inst);
            if (markers.instance_count > 0) {
                flat.prog.use();
                flat.u_use_instancing.set(1);
                flat.u_mvp.set(mvp);
                flat.u_radius.set(R);
                flat.u_inner.set(0.0f);
                flat.u_alpha.set(1.0f);
                markers.draw();
            }
        }

        // Drone dots
        flat.prog.use();
        flat.u_use_instancing.set(0);
        flat.u_radius.set(R);
        flat.u_alpha.set(1.0f);
        foreach_drone(s, [&](size_t, const Drone& d) {
            Vec<3> color = (d.kind == Drone::Kind::Player) ? Vec<3>{0.2f, 0.75f, 0.95f}
                                                           : Vec<3>{0.95f, 0.20f, 0.20f};
            Vec<2> rel = {d.position[0] - p[0], d.position[1] - p[1]};
            float dist = length(rel);
            Vec<2> marker_pos = (dist > R && dist > 0.0f) ? (R / dist) * rel : rel;
            flat.u_scale.set(12.0f);
            flat.u_translate.set(marker_pos);
            flat.u_color.set(color);
            disc.draw();
        });

        // Objective rings
        for (size_t i = 0; i < s.mission.objectives.size(); ++i) {
            if (i < s.next_objective) continue;  // already visited
            const Circle& o = s.mission.objectives[i];
            flat.u_radius.set(R);
            flat.u_scale.set(o.radius);
            Vec<2> marker_pos = {o.center[0] - p[0], o.center[1] - p[1]};
            const float dist = length(marker_pos);
            if (i <= s.next_objective + 1 && dist > R && dist > 0.0f && R > 0.0f) {
                marker_pos = (R / dist) * marker_pos;
            }
            flat.u_translate.set(marker_pos);
            Vec<3> color = (s.mission.type != Mission::Race && s.mission.type != Mission::Tutorial)
                               ? Vec<3>{0.2f, 0.9f, 0.4f}
                           : (i == s.next_objective)     ? Vec<3>{0.2f, 0.9f, 0.4f}
                           : (i == s.next_objective + 1) ? Vec<3>{0.95f, 0.85f, 0.2f}
                                                         : Vec<3>{0.5f, 0.5f, 0.5f};
            flat.u_color.set(color);
            ring.draw();
        }

        // Lock-on annulus
        if (player.lock_on > 0.0f) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            flat.u_translate.set(Vec<2>{0.0f, 0.0f});
            flat.u_scale.set(R);
            flat.u_radius.set(R);
            flat.u_inner.set(std::max(0.0f, 1.0f - player.lock_on) * R);
            flat.u_alpha.set(0.2f);
            flat.u_color.set(Vec<3>{0.9f, 0.1f, 0.1f});
            disc.draw();
            glDisable(GL_BLEND);
            flat.u_alpha.set(1.0f);
            flat.u_inner.set(0.0f);
        }

        // Radar rings
        flat.u_translate.set(Vec<2>{0.0f, 0.0f});
        flat.u_color.set(Vec<3>{0.1f, 0.4f, 0.5f});
        for (float r : {R, 2.0f * R / 3.0f, R / 3.0f}) {
            flat.u_scale.set(r);
            ring.draw();
        }

        glViewport(vp[0], vp[1], vp[2], vp[3]);
        glEnable(GL_DEPTH_TEST);
    }
};

// Word-wrap `text` into lines no wider than `max_chars` characters.
std::vector<std::string> wrap_lines(const std::string& text, uint max_chars) {
    std::vector<std::string> lines;
    for (size_t start = 0; start < text.size();) {
        size_t end = std::min(start + max_chars, text.size());
        if (end == text.size()) {
            lines.push_back(text.substr(start));
            break;
        }
        size_t cut = text.rfind(' ', end);
        if (cut == std::string::npos || cut <= start) {  // no word break: hard cut
            lines.push_back(text.substr(start, max_chars));
            start += max_chars;
        } else {
            lines.push_back(text.substr(start, cut - start));
            start = cut + 1;
            while (start < text.size() && text[start] == ' ')
                ++start;
        }
    }
    return lines;
}

struct HudView {
    GlProgram program;
    Uniform u_radius, u_pitch, u_roll;
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

        quad.set_vertices(quad2_centered, sizeof(quad2_centered), 2);
    }

    void draw(const GameState& s, TextBatch& text) {
        const Drone& d = player_drone(s);
        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        const float min_dim = std::min(float(vp[2]), float(vp[3]));
        const uint attitude_size = uint(min_dim * attitude_size_ratio);
        const uint pad = uint(min_dim * hud_pad_ratio);

        glViewport(pad, pad, attitude_size, attitude_size);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        const auto R_mat = mat_from(d.orientation);
        float pitch = std::asin(std::clamp(R_mat[2, 0], -1.0f, 1.0f));
        float roll = std::atan2(-R_mat[2, 1], R_mat[2, 2]);

        program.use();
        u_radius.set(float(attitude_size) / 2.0f);
        u_pitch.set(pitch);
        u_roll.set(roll);
        quad.draw();

        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
        glViewport(vp[0], vp[1], vp[2], vp[3]);
        glEnable(GL_DEPTH_TEST);

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
        if (s.terrain.type.kind == TerrainType::Kind::City) {
            const float lock_on_level = s.terrain.type.city.lock_on_level;
            if (d.position[2] >= lock_on_level) {
                alt_col = {1.0f, 0.2f, 0.2f};
            } else if (d.position[2] >= lock_on_level - 10.0f) {
                alt_col = {1.0f, 0.6f, 0.0f};
            }
        }
        int vs_v = int(std::round(d.velocity[2]));
        std::snprintf(buf, sizeof(buf), "ALT %3d %+1d", alt_v, vs_v);
        text.draw(buf, text_x, text_y0 + line_h * 1.0f, scale, alt_col);

        int spd_v = int(
            std::round(std::sqrt(d.velocity[0] * d.velocity[0] + d.velocity[1] * d.velocity[1])));
        std::snprintf(buf, sizeof(buf), "SPD %3d", spd_v);
        text.draw(buf, text_x, text_y0 + line_h * 2.0f, scale, {1.0f, 1.0f, 1.0f});

        const float timer_scale = 1.5f;
        const float timer_y = pad;
        if (s.mission.type == Mission::Race) {
            float sec = float(s.timer.value_or(0)) * Physics::dt;
            if (s.mission.target_time > 0) {
                std::snprintf(buf, sizeof(buf), "%.1fs (%.1fs)", sec, s.mission.target_time);
            } else {
                std::snprintf(buf, sizeof(buf), "%.1fs", sec);
            }
            Vec<3> timer_col = {1.0f, 1.0f, 1.0f};
            if (sec > s.mission.target_time)
                timer_col = {1.0f, 0.2f, 0.2f};  // overtime
            else if (s.finish_tick.has_value())
                timer_col = {0.3f, 0.9f, 0.4f};  // success
            else if (s.mission.target_time - sec <= 10.0f)
                timer_col = {1.0f, 0.9f, 0.2f};  // yellow-flag: 10s to go
            const float timer_w = float(std::strlen(buf)) * font_size[0] * timer_scale;
            const float timer_x = float(vp[2]) / 2.0f - timer_w / 2.0f;
            text.draw(buf, timer_x, timer_y, timer_scale, timer_col);
        } else if (s.mission.type == Mission::Chase) {
            float sec = float(s.timer.value_or(0)) * Physics::dt;
            std::snprintf(buf, sizeof(buf), "%.1f s (destroyed %u/%u)", sec, s.kill_count,
                          s.mission.target_kill_count);
            Vec<3> timer_col = (sec < 10.0f) ? Vec<3>{1.0f, 0.9f, 0.2f} : Vec<3>{1.0f, 1.0f, 1.0f};
            const float timer_w = float(std::strlen(buf)) * font_size[0] * timer_scale;
            const float timer_x = float(vp[2]) / 2.0f - timer_w / 2.0f;
            text.draw(buf, timer_x, timer_y, timer_scale, timer_col);
        } else if (s.mission.type == Mission::Defense) {
            float sec = float(s.timer.value_or(0)) * Physics::dt;
            std::snprintf(buf, sizeof(buf), "%u/%u LIVES (%.1fs)", s.lives,
                          s.mission.starting_lives, sec);
            Vec<3> timer_col = {1.0f, 1.0f, 1.0f};
            if (s.lives == 0)
                timer_col = {1.0f, 0.2f, 0.2f};
            else if (s.lives == 1)
                timer_col = {1.0f, 0.9f, 0.2f};
            const float timer_w = float(std::strlen(buf)) * font_size[0] * timer_scale;
            const float timer_x = float(vp[2]) / 2.0f - timer_w / 2.0f;
            text.draw(buf, timer_x, timer_y, timer_scale, timer_col);
        }

        // Tutorial: long help text wrapped across the bottom of the screen, kept
        // clear of the attitude indicator / left info column and the minimap.
        if (s.mission.type == Mission::Tutorial && !s.mission.info.empty()) {
            const float hud_col_chars = 12.0f;
            const float info_scale = 1.0f;

            const float minimap_size = min_dim * minimap_size_ratio;
            const float x_left =
                3.0f * float(pad) + float(attitude_size) + hud_col_chars * font_size[0];
            const float x_right = float(vp[2]) - minimap_size - 2 * float(pad);
            const uint max_chars =
                std::max(1u, uint((x_right - x_left) / (font_size[0] * info_scale)));

            // Line wrapping
            const auto lines = wrap_lines(s.mission.info, max_chars);
            // Render
            const float info_line_h = font_size[1] * info_scale * 1.2f;
            for (size_t i = 0; i < lines.size(); ++i) {
                const float y = float(pad) + float(lines.size() - 1 - i) * info_line_h;
                text.draw(lines[i].c_str(), x_left, y, info_scale, {0.6f, 0.9f, 0.6f});
            }
        }
    }
};

// In-game pause button (top-left).
struct PauseButtonView {
    void draw(GameState& s, Window& window, TextBatch& text, FlatShader&) {
        const float scale = 2.0f;

        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        auto w = float(vp[2]);
        auto h = float(vp[3]);
        auto pad = std::min(w, h) * hud_pad_ratio;
        auto fs = text.base_font_size();
        auto x = pad;
        auto y = h - pad - fs[1] * scale;
        auto mouse = window.mouse_pos();
        if (!s.paused && window.mouse_pressed(1) && mouse[0] >= x &&
            mouse[0] <= x + fs[0] * scale && mouse[1] >= y && mouse[1] <= y + fs[1] * scale)
            s.paused = true;
        text.draw("#", x, y, scale, {0.9f, 0.9f, 0.9f});
    }
};

struct NoiseOverlayView {
    GlProgram program;
    Uniform u_time, u_pixel_size;
    Mesh quad;
    uint time;

    NoiseOverlayView() {
        const char* v_src = R"(
#version 330 core
layout(location=0) in vec2 pos;
void main(){ gl_Position = vec4(pos * 2.0, 0.0, 1.0); }
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
        quad.set_vertices(quad2_centered, sizeof(quad2_centered), 2);
    }

    void draw(const GameState& s, TextBatch& text) {
        glDisable(GL_DEPTH_TEST);
        program.use();
        u_time.set(float(time % (1 << 20)));
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

        ++time;
    }
};

struct ObjectiveView {
    Mesh mesh;

    ObjectiveView() { mesh.set_vertices(make_cylinder(48, /*caps*/ false)); }

    void draw(const GameState& s, const FlatShader& flat, const Mat4& view_projection_mat) {
        constexpr float cyl_height = 200.0f;
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);

        flat.prog.use();
        flat.u_use_instancing.set(0);
        flat.u_radius.set(0.0f);
        flat.u_inner.set(0.0f);
        flat.u_scale.set(1.0f);
        flat.u_translate.set(Vec<2>{0, 0});

        for (size_t i = 0; i < s.mission.objectives.size(); ++i) {
            if (i < s.next_objective) continue;  // already visited
            const Circle& o = s.mission.objectives[i];
            Vec<4> color = (s.mission.type != Mission::Race && s.mission.type != Mission::Tutorial)
                               ? Vec<4>{0.2f, 0.85f, 0.4f, 0.25f}
                           : (i == s.next_objective)     ? Vec<4>{0.2f, 0.85f, 0.4f, 0.25f}
                           : (i == s.next_objective + 1) ? Vec<4>{0.95f, 0.85f, 0.2f, 0.25f}
                                                         : Vec<4>{0.5f, 0.5f, 0.5f, 0.25f};

            flat.u_color.set(Vec<3>{color[0], color[1], color[2]});
            flat.u_alpha.set(color[3]);

            Mat4 model = mat_rigid(mat_from(Quat{0.0f, 0.0f, 0.0f, 1.0f}),
                                   {o.center[0], o.center[1], cyl_height * 0.5f});
            Mat4 scale = mat_diag(o.radius * 2.0f, o.radius * 2.0f, cyl_height, 1.0f);
            Mat4 mvp = mul(view_projection_mat, mul(model, scale));
            flat.u_mvp.set(mvp);
            mesh.draw();
        }
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
};

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

// Mission briefing, shown before gameplay starts (dismissed with ENTER).
struct PreGameView {
    Mesh quad;

    PreGameView() { quad.set_vertices(quad2_centered, sizeof(quad2_centered), 2); }

    void draw(const GameState& s, const FlatShader& flat, TextBatch& text) {
        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);

        // Dim the scene behind the briefing.
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        flat.prog.use();
        flat.u_use_instancing.set(0);
        flat.u_mvp.set(mat_diag(1.0f, 1.0f, 1.0f, 1.0f));
        flat.u_radius.set(0.0f);
        flat.u_inner.set(0.0f);
        flat.u_scale.set(2.0f);
        flat.u_translate.set(Vec<2>{0.0f, 0.0f});
        flat.u_alpha.set(0.85f);
        flat.u_color.set(Vec<3>{0.02f, 0.02f, 0.05f});
        quad.draw();
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
        flat.u_alpha.set(1.0f);

        const auto font_size = text.base_font_size();
        const float cx = float(vp[2]) / 2.0f;
        const float cy = float(vp[3]) / 2.0f;
        constexpr float body_scale = 1.5f;
        const float line_h = font_size[1] * body_scale;
        auto centre = [&](const char* str, float y, float scale, const Vec<3>& color) {
            const float w = float(std::strlen(str)) * font_size[0] * scale;
            text.draw(str, cx - w / 2.0f, y, scale, color);
        };

        // Mission name
        constexpr float title_scale = 3.0f;
        const float title_w = float(s.mission.name.size()) * font_size[0] * title_scale;
        text.draw(s.mission.name.c_str(), cx - title_w / 2.0f, cy + 4.5f * line_h, title_scale,
                  {0.9f, 0.9f, 0.9f});

        // Info, word-wrapped across the centre band.
        const uint max_chars =
            std::max(1u, uint((float(vp[2]) * 0.7f) / (font_size[0] * body_scale)));
        const auto lines = wrap_lines(s.mission.info, max_chars);
        const float info_line_h = font_size[1] * body_scale * 1.2f;
        float y = cy + 2.5f * line_h;
        for (const auto& l : lines) {
            centre(l.c_str(), y, body_scale, {0.8f, 0.9f, 0.8f});
            y -= info_line_h;
        }

        // Map + drone
        char detail[96];
        std::snprintf(detail, sizeof(detail), "Map: %s    Drone: %s",
                      s.terrain.type.kind == TerrainType::Kind::City ? "city" : "plains",
                      s.mission.player_type.name.c_str());
        y -= line_h;
        centre(detail, y, body_scale, {1.0f, 1.0f, 1.0f});
        y -= 2 * line_h;

        centre("Press ENTER to continue", y, body_scale, {0.7f, 0.7f, 0.7f});
    }
};

// Simple post-game overlay, shown once a mission's end condition has been met.
struct PostGameView {
    Mesh quad;

    PostGameView() { quad.set_vertices(quad2_centered, sizeof(quad2_centered), 2); }

    void draw(const GameState& s, const FlatShader& flat, TextBatch& text) {
        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);

        // Dim the frozen scene.
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        flat.prog.use();
        flat.u_use_instancing.set(0);
        flat.u_mvp.set(mat_diag(1.0f, 1.0f, 1.0f, 1.0f));
        flat.u_radius.set(0.0f);
        flat.u_inner.set(0.0f);
        flat.u_scale.set(2.0f);
        flat.u_translate.set(Vec<2>{0.0f, 0.0f});
        flat.u_alpha.set(0.55f);
        flat.u_color.set(Vec<3>{0.02f, 0.02f, 0.05f});
        quad.draw();
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
        flat.u_alpha.set(1.0f);

        const auto font_size = text.base_font_size();
        const float cx = float(vp[2]) / 2.0f;
        const float cy = float(vp[3]) / 2.0f;
        auto centre = [&](const char* str, float y, float scale, const Vec<3>& color) {
            const float w = float(std::strlen(str)) * font_size[0] * scale;
            text.draw(str, cx - w / 2.0f, y, scale, color);
        };

        constexpr float title_scale = 3.0f;
        constexpr float line_scale = 1.5f;
        const float line_h = font_size[1] * line_scale;
        const bool failed = s.result.type == MissionResult::Failed;
        centre(failed ? "MISSION FAILED" : "MISSION COMPLETE", cy + line_h / 2 + 2 * line_h,
               title_scale, failed ? Vec<3>{1.0f, 0.3f, 0.3f} : Vec<3>{0.3f, 0.9f, 0.4f});
        char info[64];
        if (s.mission.type == Mission::Race) {
            std::snprintf(info, sizeof(info), "time: %.1fs (target %.1fs)", s.result.time,
                          s.mission.target_time);
        } else {
            std::snprintf(info, sizeof(info), "enemies destroyed: %u", s.result.score);
        }
        centre(info, cy + line_h / 2, line_scale, {1.0f, 1.0f, 1.0f});
        centre("press ENTER to continue", cy + line_h / 2 - 2 * line_h, line_scale,
               {0.7f, 0.7f, 0.7f});
    }
};

////////////////////////////////////////////////////////////////////////////////
// Menu layout & view

struct MenuView {
    struct Item {
        enum Type : uint8_t { Plain, Choice, Toggle, Slider, KeyBinding };
        Type type = Plain;
        std::string label;
        std::string value;  // Choice/KeyBinding: right-hand text, Plain: optional value
        bool enabled = true;
        bool on = false;                                   // Toggle
        float fraction = 0.0f;                             // Slider, 0..1
        int index = 0;                                     // Choice selected option index
        std::vector<std::pair<std::string, int>> options;  // Choice: (label, value)
        int data = 0;                                      // KeyBinding: Action::Type of this row
        int preset = -1;          // Controls preset: control_presets() index
        bool unlock_all = false;  // Missions "Unlock all" row
        float row_bottom = 0.0f;
    };
    const char* title = nullptr;
    const char* footer = nullptr;
    std::vector<Item> items;
    Vec<2> font_size;
    float line_h;
    float margin;

    int row_hit(Vec<2> vp, Vec<2> mouse) const {
        if (mouse[0] < margin || mouse[0] > vp[0] - margin) return -1;
        for (uint i = 0; i < items.size(); ++i) {
            if (mouse[1] >= items[i].row_bottom && mouse[1] < items[i].row_bottom + line_h)
                return int(i);
        }
        return -1;
    }
};

MenuView::Item plain_item(const char* label, bool enabled = true) {
    MenuView::Item it;
    it.type = MenuView::Item::Plain;
    it.label = label;
    it.enabled = enabled;
    return it;
}

MenuView::Item choice_item(const char* label,
                           std::vector<std::pair<std::string, int>> options,
                           int index) {
    MenuView::Item it;
    it.type = MenuView::Item::Choice;
    it.label = label;
    it.options = std::move(options);
    it.index = index;
    return it;
}

MenuView::Item toggle_item(const char* label, bool on) {
    MenuView::Item it;
    it.type = MenuView::Item::Toggle;
    it.label = label;
    it.on = on;
    return it;
}

MenuView::Item slider_item(const char* label, float fraction) {
    MenuView::Item it;
    it.type = MenuView::Item::Slider;
    it.label = label;
    it.fraction = fraction;
    return it;
}

MenuView::Item keybinding_item(const char* label, const char* value, int action) {
    MenuView::Item it;
    it.type = MenuView::Item::KeyBinding;
    it.label = label;
    it.value = value;
    it.data = action;
    return it;
}

// Empty disabled row; separates groups visually and is skipped by the cursor.
MenuView::Item spacer_item() {
    MenuView::Item it;
    it.type = MenuView::Item::Plain;
    it.enabled = false;
    return it;
}

MenuView::Item unlock_all_item() {
    MenuView::Item it;
    it.type = MenuView::Item::Plain;
    it.label = "Unlock all";
    it.unlock_all = true;
    return it;
}

MenuView::Item preset_item(const char* label, int index) {
    MenuView::Item it;
    it.type = MenuView::Item::Plain;
    it.label = label;
    it.preset = index;
    return it;
}

std::string fmt_str(float f, const char* spec) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), spec, f);
    return buf;
}

std::string fmt_int(uint n) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%u", n);
    return buf;
}

////////////////////////////////////////////////////////////////////////////////
// Menus view

struct Menus {
    static constexpr float label_scale = 1.5f;
    static constexpr float line_height_scale = 1.2f * label_scale;

    Mesh quad;
    MenuView view;
    ShellState::Screen prev_screen = ShellState::Screen::count;
    ShellState::Screen parent = ShellState::Screen::Main;
    int cursor = 0;
    int pending_binding = -1;

    // Progress-bar geometry shared by hit-testing and drawing, so every slider
    // starts/ends at the same place: 15 char widths from the left, 5 from the right.
    void slider_track(const MenuView& v, Vec<2> vp, uint i, float& x0, float& x1, float& cy) const {
        const float cw = v.font_size[0] * label_scale;
        x0 = v.margin + 15.0f * cw;
        x1 = vp[0] - v.margin - 5.0f * cw;
        cy = v.items[i].row_bottom + 0.5f * v.line_h;
    }

    void build(const ShellState& shell,
               const Settings& settings,
               const TextBatch& text,
               Vec<2> vp) {
        view = {};
        view.font_size = text.base_font_size();
        view.line_h = line_height_scale * view.font_size[1];
        view.margin = 0.15f * std::min(vp[0], vp[1]);
        switch (shell.screen) {
            case ShellState::Screen::Main:
                view.title = "QUAD INTERCEPTOR";
                view.footer = "up/down: move | enter: select | esc: quit";
                view.items.push_back(plain_item("Play"));
                view.items.push_back(plain_item("Settings"));
                view.items.push_back(plain_item("Quit"));
                break;
            case ShellState::Screen::Missions: {
                view.title = "MISSIONS";
                view.footer = "esc: back | complete a mission to unlock the next";
                auto missions = Mission::all();
                for (size_t i = 0; i < shell.missions.size(); ++i) {
                    // spacer above sandbox missions
                    if (missions[i].type == Mission::Sandbox &&
                        (i != 0 && missions[i - 1].type != Mission::Sandbox)) {
                        view.items.push_back(spacer_item());
                    }
                    const auto& ms = shell.missions[i];
                    auto it = plain_item(ms.name.c_str(), ms.unlocked);
                    it.data = int(i);
                    if (!ms.unlocked) {
                        it.value = "locked";
                    } else if (missions[i].type == Mission::Sandbox) {
                        it.value = "";
                    } else if (ms.best.type != MissionResult::Completed) {
                        it.value = "new";
                    } else if (missions[i].type == Mission::Race) {
                        it.value = fmt_str(ms.best.time, "completed (%.1fs)");
                    } else if (missions[i].type == Mission::Chase) {
                        it.value = fmt_str(ms.best.time, "completed (%.1fs left)");
                    } else if (missions[i].type == Mission::Defense) {
                        it.value = fmt_str(ms.best.score, "completed (%.0f kills)");
                    } else {
                        it.value = "completed";
                    }
                    view.items.push_back(std::move(it));
                }
                view.items.push_back(spacer_item());
                view.items.push_back(unlock_all_item());
                view.items.push_back(spacer_item());
                view.items.push_back(plain_item("Back"));
                break;
            }
            case ShellState::Screen::Pause:
                view.title = "PAUSED";
                view.footer = "esc: resume";
                view.items.push_back(plain_item("Resume"));
                view.items.push_back(plain_item("Restart"));
                view.items.push_back(plain_item("Settings"));
                view.items.push_back(spacer_item());
                view.items.push_back(plain_item("Exit to Menu"));
                break;
            case ShellState::Screen::Settings:
                view.title = "SETTINGS";
                view.footer = "esc: back";
                view.items.push_back(plain_item("Graphics"));
                view.items.push_back(plain_item("Sound"));
                view.items.push_back(plain_item("Controls"));
                view.items.push_back(spacer_item());
                view.items.push_back(plain_item("Back"));
                break;
            case ShellState::Screen::Graphics: {
                view.title = "GRAPHICS";
                view.footer = "esc: back | msaa applies on restart";
                int msaa_index = 0;
                const std::array<int, 4> modes{1, 2, 4, 8};
                for (int n = 0; n < int(modes.size()); ++n)
                    if (int(settings.msaa) == modes[n]) msaa_index = n;
                int res_index = 0;
                const auto res = Settings::resolutions();
                for (int r = 0; r < int(res.size()); ++r) {
                    if (std::get<0>(res[size_t(r)]) == settings.window_width &&
                        std::get<1>(res[size_t(r)]) == settings.window_height)
                        res_index = r;
                }
                std::vector<std::pair<std::string, int>> res_options;
                for (int r = 0; r < int(res.size()); ++r)
                    res_options.push_back({std::to_string(std::get<0>(res[size_t(r)])) + "x" +
                                               std::to_string(std::get<1>(res[size_t(r)])),
                                           r});
                view.items.push_back(choice_item("Window mode",
                                                 {{"Windowed", 0}, {"Fullscreen", 1}},
                                                 settings.fullscreen ? 1 : 0));
                view.items.push_back(
                    choice_item("Resolution (Windowed)", std::move(res_options), res_index));
                view.items.push_back(
                    choice_item("MSAA", {{"1x", 1}, {"2x", 2}, {"4x", 4}, {"8x", 8}}, msaa_index));
                view.items.push_back(toggle_item("VSync", settings.vsync));
                view.items.push_back(spacer_item());
                view.items.push_back(plain_item("Back"));
                break;
            }
            case ShellState::Screen::Sound:
                view.title = "SOUND";
                view.footer = "esc: back | left/right: adjust";
                view.items.push_back(slider_item("Master volume", settings.master_volume));
                view.items.push_back(slider_item("Music volume", settings.music_volume));
                view.items.push_back(slider_item("Effects volume", settings.effects_volume));
                view.items.push_back(spacer_item());
                view.items.push_back(plain_item("Back"));
                break;
            case ShellState::Screen::Controls: {
                view.title = "CONTROLS";
                view.footer = "esc: back | enter: rebind | duplicate bindings swap";
                for (uint a = 0; a < size_t(Action::count); ++a) {
                    if (a == 4) view.items.push_back(spacer_item());  // grouped for easier scan
                    view.items.push_back(keybinding_item(Action::name(Action::Type(a)),
                                                         scancode_name(settings.bindings[a]),
                                                         int(a)));
                }
                view.items.push_back(spacer_item());  // before the presets
                const auto presets = Settings::control_presets();
                for (uint p = 0; p < presets.size(); ++p)
                    view.items.push_back(preset_item(std::get<0>(presets[p]).c_str(), int(p)));
                view.items.push_back(spacer_item());  // after the presets
                view.items.push_back(plain_item("Back"));
                break;
            }
            default:
                gassert(false && "unexpected menu screen");
                break;
        }
        for (uint i = 0; i < view.items.size(); ++i)
            view.items[i].row_bottom = vp[1] - view.margin - float(i + 1) * view.line_h;
    }

    void enter_submenu(ShellState& shell, ShellState::Screen scr) {
        if (scr == ShellState::Screen::Settings) parent = shell.screen;  // who opened Settings
        shell.screen = scr;
    }

    // Push the focused row's value into Settings and persist; returns true if changed.
    bool apply_row(const ShellState& shell, Settings& settings, Window& window) {
        const auto& item = view.items[size_t(cursor)];
        switch (shell.screen) {
            case ShellState::Screen::Graphics:
                if (cursor == 0) {
                    settings.fullscreen = item.options[size_t(item.index)].second != 0;
                    window.set_mode(settings.fullscreen ? WindowMode::Fullscreen
                                                        : WindowMode::Windowed);
                } else if (cursor == 1) {
                    const auto res = Settings::resolutions();
                    const auto r = size_t(std::clamp(item.index, 0, int(res.size()) - 1));
                    const auto [rw, rh] = res[r];
                    settings.window_width = rw;
                    settings.window_height = rh;
                    if (settings.fullscreen) {
                        // A non-native resolution only makes sense windowed
                        const auto native = window.display_size();
                        if (std::get<0>(native) != rw || std::get<1>(native) != rh) {
                            settings.fullscreen = false;
                            window.set_mode(WindowMode::Windowed);
                        }
                    }
                    if (!settings.fullscreen) window.set_resolution(rw, rh);
                } else if (cursor == 2) {
                    settings.msaa = uint(item.options[size_t(item.index)].second);
                } else if (cursor == 3) {
                    settings.vsync = item.on;
                    window.set_vsync(item.on);
                }
                break;
            case ShellState::Screen::Sound:
                if (cursor == 0) {
                    settings.master_volume = item.fraction;
                } else if (cursor == 1) {
                    settings.music_volume = item.fraction;
                } else if (cursor == 2) {
                    settings.effects_volume = item.fraction;
                }
                break;
            default:
                return false;
        }
        settings.save(get_config_dir());
        return true;
    }

    bool adjust(const ShellState& shell, Settings& settings, Window& window, int dir) {
        auto& item = view.items[size_t(cursor)];
        switch (item.type) {
            case MenuView::Item::Choice: {
                const int n = int(item.options.size());
                if (n == 0) return false;
                item.index = (item.index + dir + n) % n;
                break;
            }
            case MenuView::Item::Toggle:
                item.on = !item.on;
                break;
            case MenuView::Item::Slider: {
                constexpr float step = 0.05f;
                item.fraction = std::clamp(item.fraction + float(dir) * step, 0.0f, 1.0f);
                break;
            }
            default:
                return false;
        }
        return apply_row(shell, settings, window);
    }

    bool activate(ShellState& shell, Settings& settings, Window& window) {
        auto& item = view.items[size_t(cursor)];
        switch (item.type) {
            case MenuView::Item::Choice:
                if (item.options.empty()) return false;
                item.index = (item.index + 1) % int(item.options.size());
                return apply_row(shell, settings, window);
            case MenuView::Item::Toggle:
                item.on = !item.on;
                return apply_row(shell, settings, window);
            case MenuView::Item::Slider:
                return false;
            case MenuView::Item::KeyBinding:
                pending_binding = item.data;  // the Action::Type, not the row index
                return false;
            case MenuView::Item::Plain:
                break;
        }
        // Plain item: navigation
        switch (shell.screen) {
            case ShellState::Screen::Main:
                if (cursor == 0) {
                    shell.screen = ShellState::Screen::Missions;
                } else if (cursor == 1) {
                    enter_submenu(shell, ShellState::Screen::Settings);
                } else if (cursor == 2) {
                    shell.navigation = ShellState::Navigation::Quit;
                }
                break;
            case ShellState::Screen::Missions: {
                const auto& m_item = view.items[size_t(cursor)];
                if (m_item.unlock_all) {
                    shell_unlock_all(shell);
                    return true;  // rebuild the view with the fresh lock state
                }
                if (cursor == int(view.items.size()) - 1) {  // Back row
                    shell.screen = ShellState::Screen::Main;
                } else {
                    const int idx = m_item.data;
                    gassert(idx >= 0 && idx < int(shell.missions.size()));
                    gassert(shell.missions[size_t(idx)].unlocked);
                    shell.chosen = Mission::all()[size_t(idx)];
                    shell.navigation = ShellState::Navigation::Play;
                }
                break;
            }
            case ShellState::Screen::Pause:
                if (cursor == 0) {
                    shell.navigation = ShellState::Navigation::Resume;
                } else if (cursor == 1) {
                    shell.navigation = ShellState::Navigation::Restart;
                } else if (cursor == 2) {
                    enter_submenu(shell, ShellState::Screen::Settings);
                } else if (cursor == 4) {
                    shell.navigation = ShellState::Navigation::Abandon;
                }
                break;
            case ShellState::Screen::Settings:
                if (cursor == 0) {
                    enter_submenu(shell, ShellState::Screen::Graphics);
                } else if (cursor == 1) {
                    enter_submenu(shell, ShellState::Screen::Sound);
                } else if (cursor == 2) {
                    enter_submenu(shell, ShellState::Screen::Controls);
                } else {
                    shell.screen = parent;  // Back
                }
                break;
            case ShellState::Screen::Graphics:
            case ShellState::Screen::Sound:
            case ShellState::Screen::Controls:
                if (cursor == int(view.items.size()) - 1)
                    shell.screen = ShellState::Screen::Settings;
                break;
            default:
                break;
        }
        if (shell.screen == ShellState::Screen::Controls && item.preset >= 0) {
            const auto presets = Settings::control_presets();
            gassert(size_t(item.preset) < presets.size());
            settings.bindings = std::get<1>(presets[size_t(item.preset)]);
            settings.save(get_config_dir());
            return true;
        }
        return false;
    }

    void back(ShellState& shell) {
        switch (shell.screen) {
            case ShellState::Screen::Main:
                shell.navigation = ShellState::Navigation::Quit;
                break;
            case ShellState::Screen::Pause:
                shell.navigation = ShellState::Navigation::Resume;
                break;
            case ShellState::Screen::Missions:
                shell.screen = ShellState::Screen::Main;
                break;
            case ShellState::Screen::Settings:
                shell.screen = parent;  // the screen that opened settings
                break;
            case ShellState::Screen::Graphics:
            case ShellState::Screen::Sound:
            case ShellState::Screen::Controls:
                shell.screen = ShellState::Screen::Settings;
                break;
            default:
                break;
        }
    }

    void update(Window& window, ShellState& shell, Settings& settings, const TextBatch& text) {
        shell.navigation = ShellState::Navigation::None;
        if (prev_screen != shell.screen) {
            prev_screen = shell.screen;
            cursor = 0;
        }

        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        const Vec<2> vp2{float(vp[2]), float(vp[3])};
        build(shell, settings, text, vp2);

        // While capturing a binding, all other input is ignored.
        if (pending_binding >= 0) {
            if (window.pressed(Key::esc)) {
                pending_binding = -1;
            } else if (const auto sc = window.scancode_pressed_any()) {
                gassert(shell.screen == ShellState::Screen::Controls);
                const auto action = Action::Type(pending_binding);
                const int old = settings.bindings[size_t(action)];
                for (uint a = 0; a < size_t(Action::count); ++a) {
                    if (settings.bindings[a] == *sc && a != size_t(action))
                        settings.bindings[a] = old;  // duplicate: swap
                }
                settings.bindings[size_t(action)] = *sc;
                pending_binding = -1;
                settings.save(get_config_dir());
            }
            build(shell, settings, text, vp2);  // show the fresh binding ("..." while pending)
            return;
        }

        // Mouse updates the cursor on move
        const int hover = view.row_hit(vp2, window.mouse_pos());
        if (window.mouse_moved() && hover >= 0 && view.items[size_t(hover)].enabled) cursor = hover;

        // Keyboard navigation skips disabled rows
        auto move_cursor = [&](int dir) {
            const int n = int(view.items.size());
            for (int step = 1; step <= n; ++step) {
                const int c = int((cursor + dir * step + n) % n);
                if (view.items[size_t(c)].enabled) {
                    cursor = c;
                    return;
                }
            }
        };
        if (window.pressed(Key::up)) move_cursor(-1);
        if (window.pressed(Key::down)) move_cursor(1);
        cursor = std::clamp(cursor, 0, std::max(0, int(view.items.size()) - 1));
        if (view.items.empty()) return;

        bool changed = false;
        if (window.pressed(Key::left)) changed = adjust(shell, settings, window, -1) || changed;
        if (window.pressed(Key::right)) changed = adjust(shell, settings, window, +1) || changed;

        // Clicking a progress bar sets its level directly, rounded to 5% increments.
        if (window.mouse_pressed(1)) {
            const Vec<2> mouse = window.mouse_pos();
            for (uint i = 0; i < view.items.size(); ++i) {
                auto& it = view.items[i];
                if (it.type != MenuView::Item::Slider || !it.enabled) continue;
                float x0, x1, cy;
                slider_track(view, vp2, i, x0, x1, cy);
                // The whole row height is clickable; only the x-range picks the level.
                const float yb = view.items[i].row_bottom;
                if (mouse[1] < yb || mouse[1] >= yb + view.line_h || mouse[0] < x0 || mouse[0] > x1)
                    continue;
                constexpr float step = 0.05f;
                it.fraction =
                    std::clamp(std::round((mouse[0] - x0) / (x1 - x0) / step) * step, 0.0f, 1.0f);
                cursor = int(i);
                changed = apply_row(shell, settings, window) || changed;
                break;
            }
        }
        if ((window.mouse_pressed(1) && hover >= 0 && hover == cursor) ||
            window.pressed(Key::enter) || window.pressed(Key::space)) {
            if (view.items[size_t(cursor)].enabled)
                changed = activate(shell, settings, window) || changed;
        }
        if (window.pressed(Key::esc)) back(shell);

        if (changed || shell.navigation != ShellState::Navigation::None ||
            prev_screen != shell.screen)
            build(shell, settings, text, vp2);
    }

    // Pause screen reminder: the actual drone model, drawn flat-white, from above
    // (pitch/roll keys around it) and from behind (lift/yaw keys around it).
    void draw_controls_reminder(TextBatch& text,
                                FlatShader& flat,
                                const Settings& settings,
                                const DroneView& drone) {
        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        const float vp_w = float(vp[2]);
        const float vp_h = float(vp[3]);
        const float u = view.line_h;

        const float span = 6.16f * u;  // drone tip-to-tip screen width
        const float gap = 0.6f * u;    // text gap beyond the props
        const float scale = 1.1f;
        const Vec<3> drone_col{0.45f, 0.55f, 0.8f};
        const Vec<3> letter_col{1.0f, 1.0f, 1.0f};

        const float cw = view.font_size[0] * scale;
        const float ch = view.font_size[1] * scale;
        const float s = span / drone.ref_width;  // model scale so the whole drone spans `span`
        const Mat4 view_mat = mat_ortho(0.0f, vp_w, 0.0f, vp_h);
        const float ex = span * 0.5f + gap;  // letter distance from centre

        auto letter = [&](const char* l, float cx, float cy) {
            const float w = float(std::strlen(l)) * cw;
            text.draw(l, cx - w / 2.0f, cy - ch / 2.0f, scale, letter_col);
        };
        auto key = [&](Action::Type a) { return scancode_name(settings.bindings[size_t(a)]); };
        auto render = [&](Vec<2> c, Quat rot, Vec<3> sc) {
            drone.draw_flat(flat, view_mat,
                            mul(mat_rigid(mat_from(rot), {c[0], c[1], 0.0f}),
                                mat_diag(sc[0], sc[1], sc[2], 1.0f)),
                            drone_col);
        };

        // Fit the pair into the band between the footer and the last menu row
        // ("Exit to Menu"), centred horizontally.
        const float band_bottom = 0.5f * view.margin + view.font_size[1];
        const float band_top =
            view.items.empty() ? vp_h - view.margin : view.items.back().row_bottom;
        const float cy = 0.5f * (band_top + band_bottom);
        const float half = ex + cw;
        const float sep =
            2.0f * half + 5.0f * u;  // centre spacing, so the side letters don't collide
        const float cx = vp_w / 2.0f;
        const Vec<2> c1{cx - 0.5f * sep, cy};
        const Vec<2> c2{cx + 0.5f * sep, cy};

        // Top-down view (drone unrotated): pitch/roll keys around it.
        // z is the depth axis and is squashed so the flat ortho keeps it clipped out.
        render(c1, Quat{0.0f, 0.0f, 0.0f, 1.0f}, {s, s, 2.0f});
        letter(key(Action::PitchFwd), c1[0], c1[1] + ex + ch / 2.0f);
        letter(key(Action::PitchBack), c1[0], c1[1] - ex - ch / 2.0f);
        letter(key(Action::RollLeft),
               c1[0] - ex - float(std::strlen(key(Action::RollLeft))) * cw / 2.0f, c1[1]);
        letter(key(Action::RollRight),
               c1[0] + ex + float(std::strlen(key(Action::RollRight))) * cw / 2.0f, c1[1]);

        // Rear view (drone rolled 90deg about x): y becomes the depth axis and is
        // squashed instead, so the two visible props line up horizontally.
        render(c2, quat_from_axis_angle({1.0f, 0.0f, 0.0f}, -pi / 2.0f), {s, 2.0f, s});
        letter(key(Action::LiftUp), c2[0], c2[1] + ex + ch / 2.0f);
        letter(key(Action::LiftDown), c2[0], c2[1] - ex - ch / 2.0f);
        letter(key(Action::YawLeft),
               c2[0] - ex - float(std::strlen(key(Action::YawLeft))) * cw / 2.0f, c2[1]);
        letter(key(Action::YawRight),
               c2[0] + ex + float(std::strlen(key(Action::YawRight))) * cw / 2.0f, c2[1]);
    }

    void draw(TextBatch& text,
              FlatShader& flat,
              bool overlay,
              const Settings& settings,
              const DroneView& drone) {
        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        const float vp_w = float(vp[2]);
        const float vp_h = float(vp[3]);

        auto draw_rect = [&](float x0, float y0, float x1, float y1, const Vec<3>& color,
                             float alpha) {
            const std::vector<Vec<2>> verts{{x0, y0}, {x1, y0}, {x1, y1},
                                            {x0, y0}, {x1, y1}, {x0, y1}};
            quad.set_vertices(verts, 2);
            flat.prog.use();
            flat.u_use_instancing.set(0);
            flat.u_radius.set(0.0f);
            flat.u_inner.set(0.0f);
            flat.u_scale.set(1.0f);
            flat.u_translate.set(Vec<2>{0.0f, 0.0f});
            flat.u_alpha.set(alpha);
            flat.u_color.set(color);
            flat.u_mvp.set(mat_ortho(0.0f, vp_w, 0.0f, vp_h));
            glDisable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            quad.draw();
            glDisable(GL_BLEND);
            glEnable(GL_DEPTH_TEST);
        };

        if (overlay) {
            draw_rect(0.0f, 0.0f, vp_w, vp_h, Vec<3>{0.02f, 0.02f, 0.04f}, 0.75f);
            if (prev_screen == ShellState::Screen::Pause)
                draw_controls_reminder(text, flat, settings, drone);
        } else {
            glClearColor(fog_color[0], fog_color[1], fog_color[2], 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        }

        const Vec<3> dim{0.45f, 0.45f, 0.45f};
        const Vec<3> lit{0.35f, 0.8f, 1.0f};

        // Cursor highlight bar (only over an enabled row)
        if (cursor >= 0 && cursor < int(view.items.size()) && view.items[size_t(cursor)].enabled) {
            const float yb = view.items[size_t(cursor)].row_bottom + 0.15f * view.line_h;
            draw_rect(view.margin, yb, vp_w - view.margin, yb + 0.7f * view.line_h,
                      Vec<3>{0.3f, 0.6f, 1.0f}, 0.12f);
        }

        // Title
        if (view.title && *view.title) {
            const float ts = 2.0f;
            const float tw = float(std::strlen(view.title)) * view.font_size[0] * ts;
            text.draw(view.title, vp_w / 2.0f - tw / 2.0f, vp_h - view.margin + view.line_h, ts,
                      {1.0f, 1.0f, 1.0f});
        }

        // Rows
        for (uint i = 0; i < view.items.size(); ++i) {
            const MenuView::Item& item = view.items[i];
            const float yb = item.row_bottom;
            const bool hover = int(i) == cursor;
            const Vec<3> label_col = !item.enabled ? dim : (hover ? lit : Vec<3>{1.0f, 1.0f, 1.0f});
            text.draw(item.label.c_str(), view.margin, yb, label_scale, label_col);

            std::string value;
            switch (item.type) {
                case MenuView::Item::Choice:
                    if (item.index >= 0 && item.index < int(item.options.size()))
                        value = item.options[size_t(item.index)].first;
                    break;
                case MenuView::Item::Toggle:
                    value = item.on ? "ON" : "OFF";
                    break;
                case MenuView::Item::KeyBinding:
                    value = (pending_binding >= 0 && item.data == pending_binding)
                                ? "< press a key >"
                                : item.value;
                    break;
                case MenuView::Item::Slider:
                    value = fmt_int(uint(std::lround(item.fraction * 100.0f))) + "%";
                    break;
                case MenuView::Item::Plain:
                    value = item.value;
                    break;
            }

            if (item.type == MenuView::Item::Slider && item.enabled) {
                float x0, x1, cy;
                slider_track(view, {vp_w, vp_h}, i, x0, x1, cy);
                const float th = 0.1f * view.line_h;
                draw_rect(x0, cy - th, x1, cy + th, Vec<3>{0.25f, 0.25f, 0.30f}, 1.0f);
                const float fill = x0 + item.fraction * (x1 - x0);
                if (fill > x0)
                    draw_rect(x0, cy - th, fill, cy + th, Vec<3>{0.35f, 0.8f, 1.0f}, 1.0f);
            }

            if (!value.empty()) {
                const float vw = float(value.size()) * view.font_size[0] * label_scale;
                const Vec<3> val_col = !item.enabled ? dim : Vec<3>{0.85f, 0.85f, 0.85f};
                text.draw(value.c_str(), vp_w - view.margin - vw, yb, label_scale, val_col);
            }
        }

        // Footer
        if (view.footer && *view.footer) {
            const float fw = float(std::strlen(view.footer)) * view.font_size[0];
            text.draw(view.footer, vp_w / 2.0f - fw / 2.0f, 0.5f * view.margin, 1.0f,
                      {0.8f, 0.8f, 0.8f});
        }
    }
};

}  // namespace

////////////////////////////////////////////////////////////////////////////////
// Renderer Top-Level

struct Renderer::State {
    // Shaders
    PhongShader phong;
    FlatShader flat;

    // Drawables
    TerrainView terrain;
    DroneView drone;
    ObjectiveView objective;
    MinimapView minimap;
    HudView hud;
    PauseButtonView pause_button;
    NoiseOverlayView overlay;
    PreGameView pregame;
    PostGameView post_game;
    FpsView fps;
    TextBatch text;

    // State
    Mat4 projection;

    // Menus
    Menus menus;

    State() : terrain(phong), drone(phong) {
        projection = mat_perspective(fov_deg * deg_to_rad, 16.0f / 9.0f, 0.1f, 1000.0f);
    }
};

Renderer::Renderer() : state(std::make_unique<State>()) {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glDisable(GL_BLEND);
}

Renderer::~Renderer() = default;

void Renderer::draw(Window& window, GameState& s, float dt) {
    glClearColor(fog_color[0], fog_color[1], fog_color[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (s.drones.empty()) return;
    const auto& player = player_drone(s);

    Mat4 view_mat = rigid_inverse(mul(mat_from(s.camera_orientation, s.camera_pos), cam_rotation));
    Mat4 view_projection_mat = mul(state->projection, view_mat);

    // 3D Scene
    state->terrain.draw(s, state->phong, view_projection_mat, s.camera_pos);
    state->drone.draw(s, state->phong, state->flat, view_projection_mat, s.camera_pos, dt);
    state->objective.draw(s, state->flat, view_projection_mat);

    // Overlays
    state->minimap.draw(s, state->flat);
    state->hud.draw(s, state->text);
    if (player.status != Drone::Status::Alive) state->overlay.draw(s, state->text);
    if (s.pregame) {
        state->pregame.draw(s, state->flat, state->text);
    } else if (s.finish_tick.has_value() && *s.finish_tick <= s.tick) {
        state->post_game.draw(s, state->flat, state->text);
    } else {
        state->pause_button.draw(s, window, state->text, state->flat);
    }
    state->fps.draw(dt, state->text);
}

void Renderer::menu_update_draw(Window& window,
                                ShellState& shell,
                                Settings& settings,
                                bool overlay) {
    state->menus.update(window, shell, settings, state->text);
    state->menus.draw(state->text, state->flat, overlay, settings, state->drone);
}
