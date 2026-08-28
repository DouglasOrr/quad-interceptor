#include "game.hpp"
#include "platform.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
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

struct Mesh {
    GlVertexArray vao;
    GlBuffer vbo;
    GlBuffer instance_vbo;
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

struct PhongShader {
    GlProgram inst_prog;
    GlProgram model_prog;

    // Uniforms
    Uniform u_inst_mvp, u_inst_light_dir, u_inst_cam_pos, u_inst_fog_color, u_inst_fog_range;
    Uniform u_model_mvp, u_model_mat, u_model_color, u_model_light_dir, u_model_cam_pos,
        u_model_fog_color, u_model_fog_range;

    PhongShader() { init(); }

    void init() {
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

        const char* inst_v_src = R"(
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
                             Vec<2> fog_range) const {
        inst_prog.use();
        u_inst_mvp.set(mvp);
        u_inst_light_dir.set(light_dir);
        u_inst_cam_pos.set(eye);
        u_inst_fog_color.set(fog_color);
        u_inst_fog_range.set(fog_range);
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
        Vec<3> A{cp[0], cp[1], b.pole.top - 2.0f};
        Vec<3> B{cc[0], cc[1], a.pole.top - 2.0f};
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

struct TerrainView {
    Mesh floor;
    Mesh buildings;
    Mesh pillars;
    WiresView wires;

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

    TerrainView(const PhongShader& phong) {
        floor.set_vertices(quad3_centered, sizeof(quad3_centered));
        floor.setup_instances_3d();

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
        const Drone& player = player_drone(s);
        const int64 ax = static_cast<int64>(std::floor(player.position[0] / cell));
        const int64 ay = static_cast<int64>(std::floor(player.position[1] / cell));

        if (ax != anchor_x || ay != anchor_y) {
            anchor_x = ax;
            anchor_y = ay;

            std::vector<Instance3D> floor_inst;
            std::vector<Instance3D> building_inst;
            std::vector<Instance3D> pillar_inst;
            wires.ends_a.clear();
            wires.ends_b.clear();

            for (int64 dy = -view_radius; dy <= view_radius; ++dy) {
                for (int64 dx = -view_radius; dx <= view_radius; ++dx) {
                    auto c = terr({ax + dx, ay + dy});
                    float wx = float(ax + dx) * cell;
                    float wy = float(ay + dy) * cell;

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
                            break;
                        }
                        case Terrain::CellType::Skywalk: {
                            const Vec<3>& g = tt.kind == TerrainType::Kind::Plains
                                                  ? plains_floor_color
                                                  : city_floor_color;
                            floor_inst.push_back({{wx, wy, 0.0f}, {cell, cell, 0.0f}, g});
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
                            const Vec<3>& g = tt.kind == TerrainType::Kind::Plains
                                                  ? plains_floor_color
                                                  : city_floor_color;
                            floor_inst.push_back({{wx, wy, 0.0f}, {cell, cell, 0.0f}, g});
                            const float r = c.pole.radius;
                            Vec<3> pole_col = (c.pole.pylon_x || c.pole.pylon_y)
                                                  ? Vec<3>{0.32f, 0.34f, 0.40f}
                                                  : Vec<3>{0.48f, 0.42f, 0.34f};
                            pillar_inst.push_back(
                                {{wx + 0.5f * cell - r, wy + 0.5f * cell - r, 0.0f},
                                 {2.0f * r, 2.0f * r, c.pole.top},
                                 pole_col});

                            if (tt.kind == TerrainType::Kind::Plains &&
                                (c.pole.pylon_x || c.pole.pylon_y)) {
                                Terrain::Pos here = {ax + dx, ay + dy};
                                const int64 stride = int64(tt.plains.pylon_stride);
                                if (c.pole.pylon_x)
                                    wires.span(terr, here, {here[0] - stride, here[1]}, true);
                                if (c.pole.pylon_y)
                                    wires.span(terr, here, {here[0], here[1] - stride}, false);
                            }
                            break;
                        }
                        case Terrain::CellType::Empty: {
                            const Vec<3>& g = tt.kind == TerrainType::Kind::Plains
                                                  ? plains_floor_color
                                                  : city_floor_color;
                            floor_inst.push_back({{wx, wy, 0.0f}, {cell, cell, 0.0f}, g});
                            break;
                        }
                    }
                }
            }
            floor.upload_instances(floor_inst);
            buildings.upload_instances(building_inst);
            pillars.upload_instances(pillar_inst);
            wires.upload();
        }

        phong.set_scene_instanced(view_projection_mat, light_dir, eye, fog_color,
                                  Vec<2>{fog_start, fog_end});

        if (floor.instance_count > 0) {
            floor_mat.use();
            floor.draw();
        }
        if (buildings.instance_count > 0) {
            building_mat.use();
            buildings.draw();
        }
        if (pillars.instance_count > 0) {
            building_mat.use();
            pillars.draw();
        }
        wires.draw(view_projection_mat, eye);
    }
};

struct DroneView {
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
                    offset[2] = d.type.height * 0.55f;
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
            const Circle& o = s.mission.objectives[i];
            flat.u_radius.set(R);
            flat.u_scale.set(o.radius);
            flat.u_translate.set(Vec<2>{o.center[0] - p[0], o.center[1] - p[1]});
            Vec<3> color = (s.mission.type != Mission::Race) ? Vec<3>{0.2f, 0.9f, 0.4f}
                           : (i == s.next_objective)         ? Vec<3>{0.2f, 0.9f, 0.4f}
                           : (i == s.next_objective + 1)     ? Vec<3>{0.95f, 0.85f, 0.2f}
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
            const Circle& o = s.mission.objectives[i];
            Vec<4> color = (s.mission.type != Mission::Race) ? Vec<4>{0.2f, 0.85f, 0.4f, 0.25f}
                           : (i == s.next_objective)         ? Vec<4>{0.2f, 0.85f, 0.4f, 0.25f}
                           : (i == s.next_objective + 1)     ? Vec<4>{0.95f, 0.85f, 0.2f, 0.25f}
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
    NoiseOverlayView overlay;
    FpsView fps;
    TextBatch text;

    // State
    Mat4 projection;

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

void Renderer::draw(const GameState& s, float dt) {
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
    state->fps.draw(dt, state->text);
}
