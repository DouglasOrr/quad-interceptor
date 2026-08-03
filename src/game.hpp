#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include "maths.hpp"

////////////////////////////////////////////////////////////////////////////////
// State
//
// Default world frame; an '_m' suffix marks a model(body)-frame quantity.

struct Pid {
    float integral = 0.0f;
    float prev = 0.0f;
    float operator()(float error, float dt);
};

struct Drone {
    inline static constexpr float height = 0.15f;

    // State
    Vec<3> position;      // (world)
    Vec<3> velocity;      // (world)
    Quat orientation;     // (model -> world)
    Vec<3> ang_momentum;  // (world)

    // Control
    Vec<4> control_m;  // (roll R/L, pitch F/B, vertical U/D, yaw CW/CCW) in [-1, 1]
    Pid pid_roll;
    Pid pid_pitch;

    static Drone create(Vec<3> position);
};

struct Cell {
    float height;  // 0=street
    uint64 hash;
};

struct City {
    inline static constexpr uint cell_size = 15;

    int64 seed;
    Cell operator()(int64 x, int64 y) const;  // (x, y) are cell coordinates
};

struct GameState {
    uint64 tick;
    std::vector<Drone> drones;  // with player = drones[0]
    City city;
};

///////////////////////////////////////////////////////////////////////////////
// Physics

struct Physics {
    inline static constexpr float dt = 1e-3f;  // seconds

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

///////////////////////////////////////////////////////////////////////////////
// Utility

template <typename... Args>
inline void print(const GameState& state, uint period, const Args&... args) {
    if (state.tick % period == 0) {
        std::cerr << '[' << state.tick << "]";
        ((std::cerr << ' ' << args), ...);
        std::cerr << '\n';
    }
}
