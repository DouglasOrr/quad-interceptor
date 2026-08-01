#pragma once

#include <format>
#include <memory>
#include <vector>

#include "maths.hpp"

////////////////////////////////////////////////////////////////////////////////
// State
//
// Default world frame; an '_m' suffix marks a model(body)-frame quantity.

struct Drone {
    inline static constexpr float height = 0.15f;

    Vec<3> position;      // (world)
    Vec<3> velocity;      // (world)
    Quat orientation;     // (model -> world)
    Vec<3> ang_momentum;  // (world)
    Vec<4> control_m;     // (right/left, forward/backward, up/down, clock/anticlock)

    static Drone create(Vec<3> position);
};

struct GameState {
    uint64 tick;
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

///////////////////////////////////////////////////////////////////////////////
// Utility

#ifdef ENABLE_LOGGING
#include <iostream>

template <typename... Args>
inline void print(const GameState& state,
                  uint period,
                  std::format_string<Args...> fmt,
                  Args&&... args) {
    if (state.tick % period == 0) {
        std::print(std::cerr, "[{:06}] {}", state.tick,
                   std::format(fmt, std::forward<Args>(args)...));
    }
}
#else
template <typename... Args>
inline void print(const GameState&, uint, std::format_string<Args...>, Args&&...) {}
#endif  // ENABLE_LOGGING
