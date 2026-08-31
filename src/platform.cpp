#include <SDL.h>
#include <algorithm>
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
    bool quit = false;
    std::array<Uint8, SDL_NUM_SCANCODES> keys{};
    std::array<Uint8, SDL_NUM_SCANCODES> prev_keys{};
    std::array<Uint8, 4> mouse{};  // index = SDL button (1..3)
    std::array<Uint8, 4> prev_mouse{};
    Vec<2> mouse_pos{};
    struct TestPress {
        SDL_Scancode sc;
        float delay;
        uint64_t trigger_time;
    };
    std::vector<TestPress> test_presses;

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
        try {
            state->audio_callback(reinterpret_cast<int16_t*>(stream), len / (2 * sizeof(int16_t)));
        } catch (...) {
            std::fprintf(stderr, "audio exception\n");
            std::memset(stream, 0, len);
        }
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

static void open_audio_device(Window::State* state) {
    SDL_AudioSpec desired{};
    desired.freq = Window::audio_sample_rate;
    desired.format = AUDIO_S16SYS;
    desired.channels = 2;
    desired.samples = 512;
    desired.callback = sdl_audio_callback;
    desired.userdata = state;

    SDL_AudioSpec obtained{};
    state->audio_dev = SDL_OpenAudioDevice(nullptr, 0, &desired, &obtained, 0);
    if (state->audio_dev != 0) {
        // 0 unpauses (starts) audio playback (devices start paused by default in SDL2)
        SDL_PauseAudioDevice(state->audio_dev, 0);
    } else {
        std::fprintf(stderr, "audio: failed to open device: %s\n", SDL_GetError());
    }
}

// Order must match the `Key` enum in platform.hpp.
static constexpr SDL_Scancode key_scancodes[] = {
    SDL_SCANCODE_W,      SDL_SCANCODE_A,      SDL_SCANCODE_S,      SDL_SCANCODE_D,
    SDL_SCANCODE_Q,      SDL_SCANCODE_E,      SDL_SCANCODE_C,      SDL_SCANCODE_SPACE,
    SDL_SCANCODE_LSHIFT, SDL_SCANCODE_LEFT,   SDL_SCANCODE_RIGHT,  SDL_SCANCODE_UP,
    SDL_SCANCODE_DOWN,   SDL_SCANCODE_X,      SDL_SCANCODE_Z,      SDL_SCANCODE_LCTRL,
    SDL_SCANCODE_LALT,   SDL_SCANCODE_COMMA,  SDL_SCANCODE_PERIOD, SDL_SCANCODE_ESCAPE,
    SDL_SCANCODE_3,      SDL_SCANCODE_RETURN,
};
static_assert(std::size(key_scancodes) == static_cast<size_t>(Key::count));

void Window::init() {
    (void)SDL_Init(0);
}

std::string Window::pref_path(const char* app_name) {
    if (char* pref = SDL_GetPrefPath(app_name, ""); pref) {
        std::string path = pref;
        SDL_free(pref);
        return path;
    }
    return "";
}

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

    SDL_GL_SetSwapInterval(settings.vsync ? 1 : 0);

    // Open audio device
    open_audio_device(state.get());
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
    if (state->quit) return false;  // quit once latched, keep reporting it
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

    // Snapshot previous inputs, then fold in this frame's events.
    state->prev_keys = state->keys;
    state->prev_mouse = state->mouse;

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            state->quit = true;
            return false;
        }
        if (event.type == SDL_AUDIODEVICEREMOVED && event.adevice.which == state->audio_dev) {
            SDL_CloseAudioDevice(state->audio_dev);
            state->audio_dev = 0;
        }
        if (event.type == SDL_AUDIODEVICEADDED && state->audio_dev == 0 &&
            !event.adevice.iscapture) {
            open_audio_device(state.get());
        }
        if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button < state->mouse.size()) {
            state->mouse[event.button.button] = 1;
        }
        if (event.type == SDL_MOUSEBUTTONUP && event.button.button < state->mouse.size()) {
            state->mouse[event.button.button] = 0;
        }
    }

    // Authoritative input state: keyboard array + mouse buttons/size.
    int numkeys = 0;
    const Uint8* kb = SDL_GetKeyboardState(&numkeys);
    for (int sc = 0; sc < SDL_NUM_SCANCODES; ++sc) {
        state->keys[sc] = (sc < numkeys) ? kb[sc] : 0;
    }

    // Test hook: fires each scheduled single-frame key press once.
    if (!state->test_presses.empty()) {
        const uint64_t freq = SDL_GetPerformanceFrequency();
        const uint64_t now = SDL_GetPerformanceCounter();
        for (auto i = 0u; i < state->test_presses.size();) {
            auto& p = state->test_presses[i];
            if (p.trigger_time == 0) {
                p.trigger_time = uint64(now + freq * p.delay);
            }
            if (now >= p.trigger_time) {
                state->keys[p.sc] = 1;
                state->test_presses.erase(state->test_presses.begin() + i);
            } else {
                ++i;
            }
        }
    }

    int mx, my;
    Uint32 mask = SDL_GetMouseState(&mx, &my);
    for (uint b = 1; b < state->mouse.size(); ++b) {
        state->mouse[b] = (mask & SDL_BUTTON(b)) ? 1 : 0;
    }
    int ww = 0, wh = 0, wd = 0, hd = 0;
    SDL_GetWindowSize(state->window, &ww, &wh);
    SDL_GL_GetDrawableSize(state->window, &wd, &hd);
    const float sx = ww > 0 ? float(wd) / float(ww) : 1.0f;
    const float sy = wh > 0 ? float(hd) / float(wh) : 1.0f;
    state->mouse_pos = {std::max(0.0f, float(mx) * sx), std::max(0.0f, float(hd) - float(my) * sy)};
    glViewport(0, 0, wd, hd);
    return true;
}

bool Window::held(Key k) const {
    const auto& st = *state;
    auto down = [&](SDL_Scancode sc) { return sc < SDL_NUM_SCANCODES && st.keys[sc]; };
    if (k == Key::hash) {
        return down(SDL_SCANCODE_3) || down(SDL_SCANCODE_NONUSHASH) || down(SDL_SCANCODE_BACKSLASH);
    }
    return down(key_scancodes[static_cast<size_t>(k)]);
}

bool Window::pressed(Key k) const {
    const auto& st = *state;
    auto pressed_sc = [&](SDL_Scancode sc) {
        return sc < SDL_NUM_SCANCODES && st.keys[sc] && !st.prev_keys[sc];
    };
    if (k != Key::hash) return pressed_sc(key_scancodes[static_cast<size_t>(k)]);
    const auto was_down = [&](SDL_Scancode sc) {
        return sc < SDL_NUM_SCANCODES && st.prev_keys[sc];
    };
    const bool now_down = held(k);
    const bool was_down_any = was_down(SDL_SCANCODE_3) || was_down(SDL_SCANCODE_NONUSHASH) ||
                              was_down(SDL_SCANCODE_BACKSLASH);
    return now_down && !was_down_any;
}

bool Window::scancode_held(int sc) const {
    return sc >= 0 && sc < SDL_NUM_SCANCODES && state->keys[sc];
}

bool Window::scancode_pressed(int sc) const {
    return sc >= 0 && sc < SDL_NUM_SCANCODES && state->keys[sc] && !state->prev_keys[sc];
}

std::optional<int> Window::scancode_pressed_any() const {
    for (int sc = 1; sc < SDL_NUM_SCANCODES; ++sc) {
        if (state->keys[sc] && !state->prev_keys[sc]) return sc;
    }
    return std::nullopt;
}

Vec<2> Window::mouse_pos() const {
    return state->mouse_pos;
}

bool Window::mouse_held(uint button) const {
    return button < state->mouse.size() && state->mouse[button];
}

bool Window::mouse_pressed(uint button) const {
    return button < state->mouse.size() && state->mouse[button] && !state->prev_mouse[button];
}

void Window::set_vsync(bool vsync) {
    SDL_GL_SetSwapInterval(vsync ? 1 : 0);
}

void Window::test_press_after(Key key, float seconds) {
    state->test_presses.push_back({key_scancodes[static_cast<size_t>(key)], seconds, 0});
}

void Window::set_mode(WindowMode mode) {
    if (mode == WindowMode::Fullscreen) {
        if (SDL_SetWindowFullscreen(state->window, SDL_WINDOW_FULLSCREEN_DESKTOP) != 0) {
            std::fprintf(stderr, "window: failed to set fullscreen: %s\n", SDL_GetError());
        }
        return;
    }
    // Windowed: the window currently spans the whole display, so shrink it to
    // 2/3 of the display resolution (otherwise it would linger at fullscreen size).
    SDL_DisplayMode dm;
    if (SDL_GetCurrentDisplayMode(0, &dm) == 0) {
        SDL_SetWindowSize(state->window, (dm.w * 2) / 3, (dm.h * 2) / 3);
    }
    if (SDL_SetWindowFullscreen(state->window, 0) != 0) {
        std::fprintf(stderr, "window: failed to leave fullscreen: %s\n", SDL_GetError());
    }
    SDL_SetWindowPosition(state->window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
}

const char* scancode_name(int sc) {
    const char* name =
        sc >= 0 && sc < SDL_NUM_SCANCODES ? SDL_GetScancodeName(SDL_Scancode(sc)) : "?";
    return *name ? name : "?";
}

void Window::set_audio_callback(std::function<void(int16_t* stream, int num_frames)> callback) {
    if (state->audio_dev != 0) SDL_LockAudioDevice(state->audio_dev);
    state->audio_callback = std::move(callback);
    if (state->audio_dev != 0) SDL_UnlockAudioDevice(state->audio_dev);
}
