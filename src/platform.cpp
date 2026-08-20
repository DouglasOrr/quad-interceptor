#include <SDL.h>
#include <array>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "platform.hpp"

#define INSTANTIATE_EXTERN(ret, name, ...) name##_t name = nullptr;
FOREACH_GL_FUNCTION(INSTANTIATE_EXTERN)
#undef INSTANTIATE_EXTERN

struct Window::State {
    SDL_Window* window = nullptr;
    SDL_GLContext gl_context = nullptr;
    SDL_AudioDeviceID audio_dev = 0;
    std::function<void(int16_t* stream, int num_frames)> audio_callback;
    bool started = false;
    std::array<bool, static_cast<size_t>(Key::count)> held_keys{};
    std::array<bool, static_cast<size_t>(Key::count)> prev_held_keys{};

    // Profiling
    std::string profile_path;
    std::vector<float> profile_data;
    uint64_t last_swap_ticks = 0;
    std::atomic<uint64_t> audio_sample_ticks{0};
    std::atomic<uint32_t> audio_samples{0};
};

namespace {
void sdl_audio_callback(void* userdata, Uint8* stream, int len) {
    auto* state = static_cast<Window::State*>(userdata);
    if (state->audio_callback) {
        const uint64_t t0 = SDL_GetPerformanceCounter();
        state->audio_callback(reinterpret_cast<int16_t*>(stream), len / (2 * sizeof(int16_t)));
        const uint64_t t1 = SDL_GetPerformanceCounter();
        if (!state->profile_path.empty()) {
            state->audio_sample_ticks.fetch_add(t1 - t0, std::memory_order_relaxed);
            state->audio_samples.fetch_add(len / (2 * sizeof(int16_t)), std::memory_order_relaxed);
        }
    } else {
        std::memset(stream, 0, len);
    }
}
}  // namespace

// Order must match the `Key` enum in platform.hpp.
static constexpr SDL_Scancode key_scancodes[] = {
    SDL_SCANCODE_W,      SDL_SCANCODE_A,      SDL_SCANCODE_S,      SDL_SCANCODE_D,
    SDL_SCANCODE_Q,      SDL_SCANCODE_E,      SDL_SCANCODE_C,      SDL_SCANCODE_SPACE,
    SDL_SCANCODE_LSHIFT, SDL_SCANCODE_LEFT,   SDL_SCANCODE_RIGHT,  SDL_SCANCODE_UP,
    SDL_SCANCODE_DOWN,   SDL_SCANCODE_X,      SDL_SCANCODE_Z,      SDL_SCANCODE_LCTRL,
    SDL_SCANCODE_COMMA,  SDL_SCANCODE_PERIOD, SDL_SCANCODE_ESCAPE, SDL_SCANCODE_3,
};
static_assert(std::size(key_scancodes) == static_cast<size_t>(Key::count));

Window::Window(const WindowSettings& settings) : state(std::make_unique<State>()) {
    state->profile_path = settings.profile_path;
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
        throw std::runtime_error(std::string("Failed to initialize SDL: ") + SDL_GetError());
    }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    // MSAA (antialiasing) via multisampled framebuffer.
    if (settings.msaa > 1) {
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, settings.msaa);
    }

    uint32_t flags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN;
    int width = settings.width;
    int height = settings.height;
    if (settings.mode == WindowMode::Fullscreen) {
        SDL_DisplayMode dm;
        if (SDL_GetCurrentDisplayMode(0, &dm) == 0) {
            width = dm.w;
            height = dm.h;
        }
        flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
    }
    state->window = SDL_CreateWindow(settings.title.c_str(), SDL_WINDOWPOS_CENTERED,
                                     SDL_WINDOWPOS_CENTERED, width, height, flags);
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

    // Disable vsync when profiling, enable otherwise
    SDL_GL_SetSwapInterval(state->profile_path.empty() ? 1 : 0);

    // Open audio device
    SDL_AudioSpec desired, obtained;
    std::memset(&desired, 0, sizeof(desired));
    desired.freq = Window::audio_sample_rate;
    desired.format = AUDIO_S16SYS;
    desired.channels = 2;
    desired.samples = 512;
    desired.callback = sdl_audio_callback;
    desired.userdata = state.get();

    state->audio_dev = SDL_OpenAudioDevice(nullptr, 0, &desired, &obtained, 0);
    if (state->audio_dev != 0) {
        // 0 unpauses (starts) audio playback (devices start paused by default in SDL2)
        SDL_PauseAudioDevice(state->audio_dev, 0);
    }
}

Window::~Window() {
    if (state->audio_dev != 0) {
        SDL_CloseAudioDevice(state->audio_dev);
    }
    if (!state->profile_data.empty()) {
        std::ofstream out(state->profile_path, std::ios::binary);
        if (out) {
            out.write("GPRF", 4);
            const uint32_t count = static_cast<uint32_t>(state->profile_data.size() / 4);
            out.write(reinterpret_cast<const char*>(&count), sizeof(count));
            out.write(reinterpret_cast<const char*>(state->profile_data.data()),
                      static_cast<std::streamsize>(state->profile_data.size() * sizeof(float)));
        }
    }
    if (state->gl_context) SDL_GL_DeleteContext(state->gl_context);
    if (state->window) SDL_DestroyWindow(state->window);
    SDL_Quit();
}

bool Window::frame() {
    if (state->started) {
        const uint64_t swap_start = SDL_GetPerformanceCounter();
        SDL_GL_SwapWindow(state->window);
        const uint64_t swap_end = SDL_GetPerformanceCounter();
        if (!state->profile_path.empty()) {
            const uint64_t freq = SDL_GetPerformanceFrequency();
            auto t_swap = static_cast<float>(swap_end - swap_start) / static_cast<float>(freq);
            auto t_frame = (state->last_swap_ticks == 0)
                               ? 0.0f
                               : static_cast<float>(swap_start - state->last_swap_ticks) /
                                     static_cast<float>(freq);
            auto t_audio = static_cast<float>(state->audio_sample_ticks.exchange(0)) /
                           static_cast<float>(freq);
            auto n_audio_samples = static_cast<float>(state->audio_samples.exchange(0));
            state->last_swap_ticks = swap_end;
            state->profile_data.push_back(t_swap);
            state->profile_data.push_back(t_frame);
            state->profile_data.push_back(t_audio);
            state->profile_data.push_back(n_audio_samples);
        }
    }
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
    int w, h;
    SDL_GL_GetDrawableSize(state->window, &w, &h);
    glViewport(0, 0, w, h);
    return true;
}

bool Window::held(Key k) const {
    return state->held_keys[static_cast<size_t>(k)];
}

bool Window::pressed(Key k) const {
    size_t idx = static_cast<size_t>(k);
    return state->held_keys[idx] && !state->prev_held_keys[idx];
}

void Window::set_audio_callback(std::function<void(int16_t* stream, int num_frames)> callback) {
    if (state->audio_dev != 0) SDL_LockAudioDevice(state->audio_dev);
    state->audio_callback = std::move(callback);
    if (state->audio_dev != 0) SDL_UnlockAudioDevice(state->audio_dev);
}
