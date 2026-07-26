#pragma once

#include <SDL_opengl.h>
#include <cstdint>
#include <memory>

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
    OP(void, glUniform3fv, GLint, GLsizei, const GLfloat*)                                  \
    OP(void, glGetProgramiv, GLuint, GLenum, GLint*)                                        \
    OP(void, glGetProgramInfoLog, GLuint, GLsizei, GLsizei*, GLchar*)

#define DECLARE_EXTERN(ret, name, ...)            \
    typedef ret(APIENTRY* name##_t)(__VA_ARGS__); \
    extern name##_t name;
FOREACH_GL_FUNCTION(DECLARE_EXTERN)
#undef DECLARE_EXTERN

struct Window {
    Window(const char* title, int width, int height);
    ~Window();

    // Swap buffers and process input events, returns false if the window should close
    bool frame();

private:
    struct State;
    std::unique_ptr<State> state;
};
