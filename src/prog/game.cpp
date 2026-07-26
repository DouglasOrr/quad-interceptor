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
    while (window.frame()) {
        glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        Physics::tick(state);  // TODO: multi-tick
        renderer.draw(state);
    }
}
