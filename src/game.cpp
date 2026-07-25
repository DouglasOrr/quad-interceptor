#include <SDL_opengl.h>

#include "platform.hpp"

int main() {
    Window window("Small Game", 1920, 1080);
    while (window.frame()) {
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
    }
    return 0;
}
