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
    state.drones = {
        Drone::create({0.0f, 0.0f, 1.0f}),
        Drone::create({10.0f, 0.0f, 1.0f}),
        Drone::create({20.0f, 5.0f, 2.0f}),
    };
    auto initial_state = state;
    Renderer renderer;
    Stopwatch stopwatch;
    float physics_time = 0.0f;
    while (window.frame()) {
        if (window.pressed(Key::hash)) {
            state = initial_state;
            physics_time = 0.0f;
        }
        copy_control(window, state.drones[0].control_m);
        physics_time = std::min(0.25f, physics_time + stopwatch.lap());
        while (physics_time >= Physics::dt) {
            Physics::tick(state);
            physics_time -= Physics::dt;
        }
        renderer.draw(state);
    }
}
