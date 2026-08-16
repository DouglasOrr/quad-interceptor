#include <cstring>

#include "game.hpp"
#include "platform.hpp"

int main(int argc, char** argv) {
    const auto missions = Mission::all();
    const Mission* selected = &missions.front();
    if (argc > 1) {
        for (const auto& m : missions)
            if (m.name == argv[1]) {
                selected = &m;
                break;
            }
    }
    Window window("Small Game", 1920, 1080);
    game_loop(window, *selected);
}
