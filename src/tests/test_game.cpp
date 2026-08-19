#include <doctest/doctest.h>

#include "game.hpp"
#include "platform.hpp"

TEST_CASE("game loop runs for a few frames") {
    Window w("Small Game", 1000, 600, WindowMode::Windowed);
    for (auto& mission : Mission::all()) {
        game_loop(w, mission, /*player_ai*/ true, /*timeout*/ 0.25f);
    }
}
