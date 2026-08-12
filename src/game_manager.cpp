#include <utility>
#include <vector>

#include "game.hpp"
#include "platform.hpp"

////////////////////////////////////////////////////////////////////////////////
// Assets

// Serialized font blob, defined in the generated font_data.cpp.
std::span<const uint8_t> font_bytes();

Font Font::load() {
    Font font;
    auto data = font_bytes();
    const uint8_t n = data[0];
    font.width = data[1];
    font.height = data[2];
    const size_t n_pixels = font.width * font.height;
    gassert(n_pixels % 8 == 0);
    const size_t cell_bytes = n_pixels / 8;
    gassert(data.size() == 3 + n * (1 + cell_bytes));
    font.glyphs.reserve(n);
    for (uint8_t i = 0; i < n; ++i) {
        const size_t off = 3 + size_t(i) * (1 + cell_bytes);
        std::vector<uint8_t> px(n_pixels);
        for (size_t p = 0; p < n_pixels; ++p)
            px[p] = (data[off + 1 + p / 8] >> (7 - p % 8)) & 1 ? 255 : 0;
        font.glyphs.push_back({data[off], std::move(px)});
    }
    return font;
}

////////////////////////////////////////////////////////////////////////////////
// GameManager

namespace {
constexpr uint64 respawn_delay = 2.0f;
constexpr Vec<3> cam_offset_vec{-1.2f, 0.0f, 0.5f};  // chase (behind, slightly up)
}  // namespace

void GameManager::update(GameState& state) {
    if (state.drones[0].status != Drone::Status::Alive) {
        if (state.respawn_time == 0)
            state.respawn_time = state.tick + uint64(respawn_delay / Physics::dt);
        if (state.tick >= state.respawn_time) {
            state.respawn_time = 0;
            state.drones[0] = Drone::create(state.spawn);
        }
    } else {
        state.respawn_time = 0;
    }

    if (!state.drones.empty()) {
        const auto& d0 = state.drones[0];
        state.camera_pos = d0.position + rotate(d0.orientation, cam_offset_vec);
        state.camera_orientation = d0.orientation;
    }
}

////////////////////////////////////////////////////////////////////////////////
// game_loop

namespace {
void copy_control(const Window& w, Vec<4>& c) {
    auto f = [](bool b) { return b ? 1.0f : 0.0f; };
    c = {};
    // Arrows
    c[0] += f(w.held(Key::right)) - f(w.held(Key::left));
    c[1] += f(w.held(Key::up)) - f(w.held(Key::down));
    // ZXC
    c[2] += 2 * f(w.held(Key::x)) - 1;
    c[3] += f(w.held(Key::c)) - f(w.held(Key::z));
}
}  // namespace

void game_loop(Window& window) {
    GameState state;
    auto C = float(City::cell_size);
    state.spawn = {C, C, 1.0f};
    state.drones = {
        Drone::create(state.spawn),
        Drone::create({5 * C, C, 1.0f}),
        Drone::create({C, 10 * C, 1.0f}),
    };
    auto initial_state = state;
    GameManager game_manager;
    Renderer renderer;
    AudioEngine audio_engine;
    window.set_audio_callback([&audio_engine](int16_t* stream, int num_frames) {
        audio_engine.fill_buffer(stream, num_frames);
    });
    Stopwatch stopwatch;
    float physics_time = 0.0f;
    while (window.frame()) {
        const float dt = stopwatch.lap();
        if (window.pressed(Key::hash)) {
            state = initial_state;
            physics_time = 0.0f;
        }
        copy_control(window, state.drones[0].control_m);
        physics_time = std::min(0.25f, physics_time + dt);
        while (physics_time >= Physics::dt) {
            Physics::tick(state);
            physics_time -= Physics::dt;
        }
        game_manager.update(state);
        renderer.draw(state, dt);
        audio_engine.update(state);
    }
}
