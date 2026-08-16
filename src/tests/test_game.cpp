#include <doctest/doctest.h>

#include <SDL.h>

#include "game.hpp"
#include "platform.hpp"

namespace {
Uint32 esc_callback(Uint32, void*) {
    SDL_Event e{};
    e.type = SDL_KEYDOWN;
    e.key.keysym.sym = SDLK_ESCAPE;
    SDL_PushEvent(&e);
    return 0;
}
}  // namespace

TEST_CASE("game loop runs for a few frames") {
    Window w("Small Game", 1000, 600);
    for (auto& mission : Mission::all()) {
        const SDL_TimerID timer = SDL_AddTimer(250, esc_callback, nullptr);
        game_loop(w, mission);
        SDL_RemoveTimer(timer);
        SDL_FlushEvents(SDL_KEYDOWN, SDL_KEYDOWN);
    }
}
