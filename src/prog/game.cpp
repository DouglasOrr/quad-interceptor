#include "game.hpp"
#include "platform.hpp"

namespace {
void copy_control(const Window& w, Vec<4>& c) {
    auto f = [](bool b) { return b ? 1.0f : 0.0f; };
    c[0] = f(w.held(Key::d)) + f(w.held(Key::right)) - f(w.held(Key::a)) - f(w.held(Key::left));
    c[1] = f(w.held(Key::w)) + f(w.held(Key::up)) - f(w.held(Key::s)) - f(w.held(Key::down));
    c[2] = f(w.held(Key::shift)) + f(w.held(Key::x)) - f(w.held(Key::ctrl)) - f(w.held(Key::z));
    c[3] = f(w.held(Key::e)) + f(w.held(Key::period)) - f(w.held(Key::q)) - f(w.held(Key::comma));
}
}  // namespace

int main() {
    Window window("Small Game", 1920, 1080);

    GameState state;
    state.drones = {
        Drone{.position = {0.0f, 0.0f, 0.0f},
              .velocity = {},
              .orientation = {0.0f, 0.0f, 0.0f, 1.0f},
              .control = {}},
        Drone{.position = {3.0f, 0.0f, 0.0f},
              .velocity = {},
              .orientation = {0.0f, 0.0f, 0.0f, 1.0f},
              .control = {}},
    };
    Renderer renderer;
    Stopwatch stopwatch;
    float physics_time = 0.0f;

    while (window.frame()) {
        copy_control(window, state.drones[0].control);
        physics_time = std::min(0.25f, physics_time + stopwatch.lap());
        while (physics_time >= Physics::dt) {
            Physics::tick(state);
            physics_time -= Physics::dt;
        }
        renderer.draw(state);
    }
}
