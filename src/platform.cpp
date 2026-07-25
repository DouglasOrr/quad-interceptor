#include <SDL.h>
#include <stdexcept>

#include "platform.hpp"

#define INSTANTIATE_EXTERN(ret, name, ...) name##_t name = nullptr;
FOREACH_GL_FUNCTION(INSTANTIATE_EXTERN)
#undef INSTANTIATE_EXTERN

struct Window::State {
    SDL_Window* window = nullptr;
    SDL_GLContext gl_context = nullptr;
    bool started = false;
};

Window::Window(const char* title, int width, int height) : state_(std::make_unique<State>()) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        throw std::runtime_error(std::string("Failed to initialize SDL: ") + SDL_GetError());
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    state_->window = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width,
                                      height, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    if (!state_->window) {
        std::string err = std::string("Failed to create window: ") + SDL_GetError();
        SDL_Quit();
        throw std::runtime_error(err);
    }

    state_->gl_context = SDL_GL_CreateContext(state_->window);
    if (!state_->gl_context) {
        std::string err = std::string("Failed to create GL context: ") + SDL_GetError();
        SDL_DestroyWindow(state_->window);
        SDL_Quit();
        throw std::runtime_error(err);
    }

#define LOAD_GL(ret, name, ...) name = (name##_t)SDL_GL_GetProcAddress(#name);
    FOREACH_GL_FUNCTION(LOAD_GL)
#undef LOAD_GL

    SDL_GL_SetSwapInterval(1);
}

Window::~Window() {
    if (state_->gl_context) SDL_GL_DeleteContext(state_->gl_context);
    if (state_->window) SDL_DestroyWindow(state_->window);
    SDL_Quit();
}

bool Window::frame() {
    if (state_->started) SDL_GL_SwapWindow(state_->window);
    state_->started = true;

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) return false;
        if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) return false;
    }
    return true;
}
