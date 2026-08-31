#pragma once

#include <SDL.h>
#include <SDL_opengl.h>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

#include "maths.hpp"

#define FOREACH_GL_FUNCTION(OP)                                                             \
    OP(GLuint, glCreateShader, GLenum)                                                      \
    OP(void, glShaderSource, GLuint, GLsizei, const GLchar* const*, const GLint*)           \
    OP(void, glCompileShader, GLuint)                                                       \
    OP(GLuint, glCreateProgram)                                                             \
    OP(void, glAttachShader, GLuint, GLuint)                                                \
    OP(void, glLinkProgram, GLuint)                                                         \
    OP(void, glUseProgram, GLuint)                                                          \
    OP(void, glGenVertexArrays, GLsizei, GLuint*)                                           \
    OP(void, glBindVertexArray, GLuint)                                                     \
    OP(void, glGenBuffers, GLsizei, GLuint*)                                                \
    OP(void, glBindBuffer, GLenum, GLuint)                                                  \
    OP(void, glBufferData, GLenum, GLsizeiptr, const void*, GLenum)                         \
    OP(void, glVertexAttribPointer, GLuint, GLint, GLenum, GLboolean, GLsizei, const void*) \
    OP(void, glEnableVertexAttribArray, GLuint)                                             \
    OP(void, glGetShaderiv, GLuint, GLenum, GLint*)                                         \
    OP(void, glGetShaderInfoLog, GLuint, GLsizei, GLsizei*, GLchar*)                        \
    OP(void, glDeleteShader, GLuint)                                                        \
    OP(void, glDeleteProgram, GLuint)                                                       \
    OP(void, glDeleteBuffers, GLsizei, GLuint*)                                             \
    OP(void, glDeleteVertexArrays, GLsizei, GLuint*)                                        \
    OP(GLint, glGetUniformLocation, GLuint, const GLchar*)                                  \
    OP(void, glUniformMatrix4fv, GLint, GLsizei, GLboolean, const GLfloat*)                 \
    OP(void, glUniform1f, GLint, GLfloat)                                                   \
    OP(void, glUniform3fv, GLint, GLsizei, const GLfloat*)                                  \
    OP(void, glUniform2fv, GLint, GLsizei, const GLfloat*)                                  \
    OP(void, glUniform4fv, GLint, GLsizei, const GLfloat*)                                  \
    OP(void, glGetProgramiv, GLuint, GLenum, GLint*)                                        \
    OP(void, glGetProgramInfoLog, GLuint, GLsizei, GLsizei*, GLchar*)                       \
    OP(void, glDrawArraysInstanced, GLenum, GLint, GLsizei, GLsizei)                        \
    OP(void, glVertexAttribDivisor, GLuint, GLuint)                                         \
    OP(void, glUniform1i, GLint, GLint)

#define DECLARE_EXTERN(ret, name, ...)            \
    typedef ret(APIENTRY* name##_t)(__VA_ARGS__); \
    extern name##_t name;
FOREACH_GL_FUNCTION(DECLARE_EXTERN)
#undef DECLARE_EXTERN

enum class WindowMode : uint8_t { Windowed, Fullscreen };

enum class Key : uint8_t {
    w,
    a,
    s,
    d,
    q,
    e,
    c,
    space,
    shift,
    left,
    right,
    up,
    down,
    x,
    z,
    ctrl,
    alt,
    comma,
    period,
    esc,
    hash,
    enter,
    count,
};

struct WindowSettings {
    std::string title;
    WindowMode mode;
    int width;
    int height;
    uint msaa;
    bool vsync;
    std::string profile_path;
};

struct Window {
    struct State;
    inline static constexpr int audio_sample_rate = 44100;

    static void init();
    static std::string pref_path(const char* app_name);  // user-preferences dir, or "" on failure

    Window(const WindowSettings& settings);
    ~Window();

    // Swap buffers and process input events, returns false if the window should close
    bool frame();

    // Query a key's held state as of the most recent frame()
    bool held(Key k) const;

    // Query whether a key was pressed (transition from not held to held) in the most recent frame()
    bool pressed(Key k) const;

    // Raw scancode queries (values are SDL_Scancode)
    bool scancode_held(int sc) const;
    bool scancode_pressed(int sc) const;
    // First scancode transitioned to held this frame, if any
    std::optional<int> scancode_pressed_any() const;

    // Mouse position in drawable pixels, y-up (to match GL), + button state
    Vec<2> mouse_pos() const;
    bool mouse_held(uint button) const;
    bool mouse_pressed(uint button) const;

    void set_vsync(bool vsync);
    void set_mode(WindowMode mode);

    void set_audio_callback(std::function<void(int16_t* stream, int num_frames)> callback);

    void test_press_after(Key key, float seconds);  // keyboard injection for tests

private:
    std::unique_ptr<State> state;
};

const char* scancode_name(int sc);  // e.g. "Return", "Left"

struct Stopwatch {
    uint64_t start;
    uint64_t freq;

    Stopwatch() : start(SDL_GetPerformanceCounter()), freq(SDL_GetPerformanceFrequency()) {}

    float lap() {
        const uint64_t now = SDL_GetPerformanceCounter();
        auto result = static_cast<float>(now - start) / static_cast<float>(freq);
        start = now;
        return result;
    }
};
