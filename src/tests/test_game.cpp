#include <doctest/doctest.h>

#include "game.hpp"
#include "platform.hpp"

TEST_CASE("game loop runs for a few frames") {
    Window w({
        .title = "Test (Small Game)",
        .mode = WindowMode::Windowed,
        .width = 1000,
        .height = 600,
        .msaa = 1,
        .profile_path = "",
    });
    for (auto& mission : Mission::all()) {
        game_loop(w, mission, /*player_ai*/ true, /*timeout*/ 0.25f);
    }
}
