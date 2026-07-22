#include <SDL.h>
#include <SDL_opengl.h>
#include <iostream>

// OpenGL Function Pointer Types
typedef GLuint(APIENTRY* PFN_glCreateShader)(GLenum type);
typedef void(APIENTRY* PFN_glShaderSource)(GLuint shader,
                                           GLsizei count,
                                           const GLchar* const* string,
                                           const GLint* length);
typedef void(APIENTRY* PFN_glCompileShader)(GLuint shader);
typedef void(APIENTRY* PFN_glGetShaderiv)(GLuint shader, GLenum pname, GLint* params);
typedef void(APIENTRY* PFN_glGetShaderInfoLog)(GLuint shader,
                                               GLsizei bufSize,
                                               GLsizei* length,
                                               GLchar* infoLog);
typedef GLuint(APIENTRY* PFN_glCreateProgram)(void);
typedef void(APIENTRY* PFN_glAttachShader)(GLuint program, GLuint shader);
typedef void(APIENTRY* PFN_glLinkProgram)(GLuint program);
typedef void(APIENTRY* PFN_glGetProgramiv)(GLuint program, GLenum pname, GLint* params);
typedef void(APIENTRY* PFN_glGetProgramInfoLog)(GLuint program,
                                                GLsizei bufSize,
                                                GLsizei* length,
                                                GLchar* infoLog);
typedef void(APIENTRY* PFN_glUseProgram)(GLuint program);
typedef void(APIENTRY* PFN_glGenVertexArrays)(GLsizei n, GLuint* arrays);
typedef void(APIENTRY* PFN_glBindVertexArray)(GLuint array);
typedef void(APIENTRY* PFN_glGenBuffers)(GLsizei n, GLuint* buffers);
typedef void(APIENTRY* PFN_glBindBuffer)(GLenum target, GLuint buffer);
typedef void(APIENTRY* PFN_glBufferData)(GLenum target,
                                         GLsizeiptr size,
                                         const void* data,
                                         GLenum usage);
typedef void(APIENTRY* PFN_glVertexAttribPointer)(GLuint index,
                                                  GLint size,
                                                  GLenum type,
                                                  GLboolean normalized,
                                                  GLsizei stride,
                                                  const void* pointer);
typedef void(APIENTRY* PFN_glEnableVertexAttribArray)(GLuint index);
typedef void(APIENTRY* PFN_glDeleteShader)(GLuint shader);
typedef void(APIENTRY* PFN_glDeleteProgram)(GLuint program);
typedef void(APIENTRY* PFN_glDeleteBuffers)(GLsizei n, const GLuint* buffers);
typedef void(APIENTRY* PFN_glDeleteVertexArrays)(GLsizei n, const GLuint* arrays);

// Function Pointer Instances
PFN_glCreateShader glCreateShader = nullptr;
PFN_glShaderSource glShaderSource = nullptr;
PFN_glCompileShader glCompileShader = nullptr;
PFN_glGetShaderiv glGetShaderiv = nullptr;
PFN_glGetShaderInfoLog glGetShaderInfoLog = nullptr;
PFN_glCreateProgram glCreateProgram = nullptr;
PFN_glAttachShader glAttachShader = nullptr;
PFN_glLinkProgram glLinkProgram = nullptr;
PFN_glGetProgramiv glGetProgramiv = nullptr;
PFN_glGetProgramInfoLog glGetProgramInfoLog = nullptr;
PFN_glUseProgram glUseProgram = nullptr;
PFN_glGenVertexArrays glGenVertexArrays = nullptr;
PFN_glBindVertexArray glBindVertexArray = nullptr;
PFN_glGenBuffers glGenBuffers = nullptr;
PFN_glBindBuffer glBindBuffer = nullptr;
PFN_glBufferData glBufferData = nullptr;
PFN_glVertexAttribPointer glVertexAttribPointer = nullptr;
PFN_glEnableVertexAttribArray glEnableVertexAttribArray = nullptr;
PFN_glDeleteShader glDeleteShader = nullptr;
PFN_glDeleteProgram glDeleteProgram = nullptr;
PFN_glDeleteBuffers glDeleteBuffers = nullptr;
PFN_glDeleteVertexArrays glDeleteVertexArrays = nullptr;

// Helper to load modern OpenGL functions dynamically via SDL
bool loadGLFunctions() {
#define LOAD_PROC(type, name)                                                  \
    name = (type)SDL_GL_GetProcAddress(#name);                                 \
    if (!name) {                                                               \
        std::cerr << "Failed to load OpenGL function: " << #name << std::endl; \
        return false;                                                          \
    }

    LOAD_PROC(PFN_glCreateShader, glCreateShader);
    LOAD_PROC(PFN_glShaderSource, glShaderSource);
    LOAD_PROC(PFN_glCompileShader, glCompileShader);
    LOAD_PROC(PFN_glGetShaderiv, glGetShaderiv);
    LOAD_PROC(PFN_glGetShaderInfoLog, glGetShaderInfoLog);
    LOAD_PROC(PFN_glCreateProgram, glCreateProgram);
    LOAD_PROC(PFN_glAttachShader, glAttachShader);
    LOAD_PROC(PFN_glLinkProgram, glLinkProgram);
    LOAD_PROC(PFN_glGetProgramiv, glGetProgramiv);
    LOAD_PROC(PFN_glGetProgramInfoLog, glGetProgramInfoLog);
    LOAD_PROC(PFN_glUseProgram, glUseProgram);
    LOAD_PROC(PFN_glGenVertexArrays, glGenVertexArrays);
    LOAD_PROC(PFN_glBindVertexArray, glBindVertexArray);
    LOAD_PROC(PFN_glGenBuffers, glGenBuffers);
    LOAD_PROC(PFN_glBindBuffer, glBindBuffer);
    LOAD_PROC(PFN_glBufferData, glBufferData);
    LOAD_PROC(PFN_glVertexAttribPointer, glVertexAttribPointer);
    LOAD_PROC(PFN_glEnableVertexAttribArray, glEnableVertexAttribArray);
    LOAD_PROC(PFN_glDeleteShader, glDeleteShader);
    LOAD_PROC(PFN_glDeleteProgram, glDeleteProgram);
    LOAD_PROC(PFN_glDeleteBuffers, glDeleteBuffers);
    LOAD_PROC(PFN_glDeleteVertexArrays, glDeleteVertexArrays);

#undef LOAD_PROC
    return true;
}

// Shaders
const char* vertexShaderSource = R"glsl(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aColor;
    out vec3 ourColor;
    void main() {
        gl_Position = vec4(aPos, 1.0);
        ourColor = aColor;
    }
)glsl";

const char* fragmentShaderSource = R"glsl(
    #version 330 core
    out vec4 FragColor;
    in vec3 ourColor;
    void main() {
        FragColor = vec4(ourColor, 1.0);
    }
)glsl";

bool checkShaderCompile(GLuint shader) {
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        GLchar infoLog[512];
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "Shader Compilation Error:\n" << infoLog << std::endl;
        return false;
    }
    return true;
}

bool checkProgramLink(GLuint program) {
    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        GLchar infoLog[512];
        glGetProgramInfoLog(program, 512, nullptr, infoLog);
        std::cerr << "Program Linking Error:\n" << infoLog << std::endl;
        return false;
    }
    return true;
}

int main(int, char**) {
    // Initialize SDL Video
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl;
        return -1;
    }

    // Request OpenGL 3.3 Core Profile
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    // Create Window
    SDL_Window* window =
        SDL_CreateWindow("OpenGL Colored Triangle", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                         800, 600, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);

    if (!window) {
        std::cerr << "Failed to create SDL Window: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return -1;
    }

    // Create OpenGL Context
    SDL_GLContext glContext = SDL_GL_CreateContext(window);
    if (!glContext) {
        std::cerr << "Failed to create OpenGL Context: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }

    // Load modern OpenGL functions
    if (!loadGLFunctions()) {
        SDL_GL_DeleteContext(glContext);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }

    // Use VSync
    SDL_GL_SetSwapInterval(1);

    // Compile Vertex Shader
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
    glCompileShader(vertexShader);
    if (!checkShaderCompile(vertexShader)) return -1;

    // Compile Fragment Shader
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
    glCompileShader(fragmentShader);
    if (!checkShaderCompile(fragmentShader)) return -1;

    // Link Shader Program
    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    if (!checkProgramLink(shaderProgram)) return -1;

    // Clean up shaders as they are linked into the program now
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // Triangle Vertices (Positions and Colors interleaved)
    float vertices[] = {
        // Positions         // Colors
        0.0f,  0.5f,  0.0f, 1.0f, 0.0f, 0.0f,  // Top (Red)
        -0.5f, -0.5f, 0.0f, 0.0f, 1.0f, 0.0f,  // Bottom Left (Green)
        0.5f,  -0.5f, 0.0f, 0.0f, 0.0f, 1.0f   // Bottom Right (Blue)
    };

    // Setup VAO and VBO
    GLuint VAO, VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // Position Attribute (Location 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Color Attribute (Location 1)
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // Unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // Main Loop
    bool running = true;
    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running = false;
                }
            }
        }

        // Render
        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw Triangle
        glUseProgram(shaderProgram);
        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, 3);

        // Swap Buffers
        SDL_GL_SwapWindow(window);
    }

    // Cleanup
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);

    SDL_GL_DeleteContext(glContext);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
