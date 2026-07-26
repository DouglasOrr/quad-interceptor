#pragma once

#include <array>
#include <vector>

using uint = unsigned int;

template <uint N>
using Vec = std::array<float, N>;
using Quat = Vec<4>;  // (x, y, z, w) where w is the scalar part

///////////////////////////////////////////////////////////////////////////////
// State

struct Drone {
    Vec<3> position;
    Vec<3> velocity;
    Quat orientation;
    Vec<4> control;  // (left/right, forward/backward, up/down, yaw)
};

struct GameState {
    std::vector<Drone> drones;  // with player = drones[0]
};

///////////////////////////////////////////////////////////////////////////////
// Physics

struct Physics {
    inline static constexpr float dt = 1e-3f;  // seconds per tick

    static void tick(GameState& state);
};
