#pragma once

#include <memory>
#include <vector>

#include "maths.hpp"

////////////////////////////////////////////////////////////////////////////////
// State

struct Drone {
    // static properties
    float mass;
    Mat<3, 3> inv_inertia;  // (model)

    // dynamic state
    Vec<3> position;      // (world)
    Vec<3> velocity;      // (world)
    Quat orientation;     // (model -> world)
    Vec<3> ang_momentum;  // (world)
    Vec<4> control;       // (right/left, forward/backward, up/down, clock/anticlock)

    static Drone create(Vec<3> position);
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

///////////////////////////////////////////////////////////////////////////////
// Renderer

struct Renderer {
    Renderer();
    ~Renderer();

    void draw(const GameState& state);

private:
    struct State;
    std::unique_ptr<State> state;
};
