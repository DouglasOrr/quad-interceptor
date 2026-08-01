#include <SDL.h>
#include <array>
#include <stdexcept>

#include "platform.hpp"

#define INSTANTIATE_EXTERN(ret, name, ...) name##_t name = nullptr;
FOREACH_GL_FUNCTION(INSTANTIATE_EXTERN)
#undef INSTANTIATE_EXTERN

struct Window::State {
    SDL_Window* window = nullptr;
    SDL_GLContext gl_context = nullptr;
    bool started = false;
    std::array<bool, static_cast<size_t>(Key::count)> held_keys{};
    std::array<bool, static_cast<size_t>(Key::count)> prev_held_keys{};
};

// Order must match the `Key` enum in platform.hpp.
static constexpr SDL_Scancode key_scancodes[] = {
    SDL_SCANCODE_W,      SDL_SCANCODE_A,      SDL_SCANCODE_S,      SDL_SCANCODE_D,
    SDL_SCANCODE_Q,      SDL_SCANCODE_E,      SDL_SCANCODE_C,      SDL_SCANCODE_SPACE,
    SDL_SCANCODE_LSHIFT, SDL_SCANCODE_LEFT,   SDL_SCANCODE_RIGHT,  SDL_SCANCODE_UP,
    SDL_SCANCODE_DOWN,   SDL_SCANCODE_X,      SDL_SCANCODE_Z,      SDL_SCANCODE_LCTRL,
    SDL_SCANCODE_COMMA,  SDL_SCANCODE_PERIOD, SDL_SCANCODE_ESCAPE, SDL_SCANCODE_3,
};
static_assert(std::size(key_scancodes) == static_cast<size_t>(Key::count));

Window::Window(const char* title, int width, int height) : state(std::make_unique<State>()) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        throw std::runtime_error(std::string("Failed to initialize SDL: ") + SDL_GetError());
    }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    // MSAA (antialiasing) via multisampled framebuffer.
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
    state->window = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width,
                                     height, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    if (!state->window) {
        std::string err = std::string("Failed to create window: ") + SDL_GetError();
        SDL_Quit();
        throw std::runtime_error(err);
    }
    state->gl_context = SDL_GL_CreateContext(state->window);
    if (!state->gl_context) {
        std::string err = std::string("Failed to create GL context: ") + SDL_GetError();
        SDL_DestroyWindow(state->window);
        SDL_Quit();
        throw std::runtime_error(err);
    }
#define LOAD_GL(ret, name, ...) name = (name##_t)SDL_GL_GetProcAddress(#name);
    FOREACH_GL_FUNCTION(LOAD_GL)
#undef LOAD_GL

    // Set defaults
    int w, h;
    SDL_GL_GetDrawableSize(state->window, &w, &h);
    glViewport(0, 0, w, h);
    SDL_GL_SetSwapInterval(1);
}

Window::~Window() {
    if (state->gl_context) SDL_GL_DeleteContext(state->gl_context);
    if (state->window) SDL_DestroyWindow(state->window);
    SDL_Quit();
}

bool Window::frame() {
    if (state->started) SDL_GL_SwapWindow(state->window);
    state->started = true;

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) return false;
        if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) return false;
    }

    state->prev_held_keys = state->held_keys;

    int numkeys = 0;
    const Uint8* kb = SDL_GetKeyboardState(&numkeys);
    for (size_t i = 0; i < state->held_keys.size(); ++i) {
        SDL_Scancode sc = key_scancodes[i];
        if (sc == SDL_SCANCODE_3) {
            state->held_keys[i] =
                (sc < numkeys && kb[sc]) ||
                (SDL_SCANCODE_NONUSHASH < numkeys && kb[SDL_SCANCODE_NONUSHASH]) ||
                (SDL_SCANCODE_BACKSLASH < numkeys && kb[SDL_SCANCODE_BACKSLASH]);
        } else {
            state->held_keys[i] = sc < numkeys && kb[sc];
        }
    }
    return true;
}

bool Window::held(Key k) const {
    return state->held_keys[static_cast<size_t>(k)];
}

bool Window::pressed(Key k) const {
    size_t idx = static_cast<size_t>(k);
    return state->held_keys[idx] && !state->prev_held_keys[idx];
}
