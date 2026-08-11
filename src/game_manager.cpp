#include "game.hpp"

namespace {
constexpr uint64 respawn_delay = 2.0f;
constexpr Vec<3> cam_offset_vec{-1.2f, 0.0f, 0.5f};  // chase (behind, slightly up)
}  // namespace

void GameManager::update(GameState& state) {
    if (state.drones[0].status != Drone::Status::Alive) {
        if (state.respawn_time == 0) state.respawn_time = state.tick + respawn_delay / Physics::dt;
        if (state.tick >= state.respawn_time) {
            state.respawn_time = 0;
            state.drones[0] = Drone::create(state.spawn);
        }
    } else {
        state.respawn_time = 0;
    }

    if (!state.drones.empty()) {
        const auto& d0 = state.drones[0];
        state.camera_pos = d0.position + rotate(d0.orientation, cam_offset_vec);
        state.camera_orientation = d0.orientation;
    }
}
