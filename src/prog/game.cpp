#include "game.hpp"
#include "platform.hpp"

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
        physics_time = std::min(0.25f, physics_time + stopwatch.lap());
        while (physics_time >= Physics::dt) {
            Physics::tick(state);
            physics_time -= Physics::dt;
        }
        renderer.draw(state);
    }
}
