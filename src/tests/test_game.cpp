#include <doctest/doctest.h>

#include <SDL.h>

#include "game.hpp"
#include "platform.hpp"

namespace {
Uint32 quit_callback(Uint32, void*) {
    SDL_Event quit{};
    quit.type = SDL_QUIT;
    SDL_PushEvent(&quit);
    return 0;
}
}  // namespace

TEST_CASE("game loop runs for a few frames") {
    Window w("Small Game", 1000, 600);
    const SDL_TimerID timer = SDL_AddTimer(250, quit_callback, nullptr);
    game_loop(w);
    SDL_RemoveTimer(timer);
    SDL_FlushEvents(SDL_QUIT, SDL_QUIT);
}
