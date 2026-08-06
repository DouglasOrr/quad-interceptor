#include "game.hpp"

namespace {
constexpr uint64 RESPAWN_TIME = 2.0f;
}  // namespace

void GameManager::update(GameState& state) {
    if (!state.drones[0].alive) {
        if (state.respawn_time == 0)
            state.respawn_time = state.tick + RESPAWN_TIME / Physics::dt;
        if (state.tick >= state.respawn_time) {
            state.respawn_time = 0;
            state.drones[0] = Drone::create(state.spawn);
        }
    } else {
        state.respawn_time = 0;
    }
}
