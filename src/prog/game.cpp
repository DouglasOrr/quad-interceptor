#include "game.hpp"
#include "platform.hpp"

namespace {
void copy_control(const Window& w, Vec<4>& c) {
    auto f = [](bool b) { return b ? 1.0f : 0.0f; };
    c = {};
    // WSAD
    c[0] += f(w.held(Key::d)) - f(w.held(Key::a));
    c[1] += f(w.held(Key::w)) - f(w.held(Key::s));
    // Arrows
    c[0] += f(w.held(Key::right)) - f(w.held(Key::left));
    c[1] += f(w.held(Key::up)) - f(w.held(Key::down));
    // ZXC
    c[2] += f(w.held(Key::x));
    c[3] += f(w.held(Key::c)) - f(w.held(Key::z));
    // QEShift
    c[2] += f(w.held(Key::shift));
    c[3] += f(w.held(Key::e)) - f(w.held(Key::q));
}
}  // namespace

int main() {
    Window window("Small Game", 1920, 1080);

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
        audio_engine.update(state, renderer.camera_pos(), renderer.camera_right());
    }
}
