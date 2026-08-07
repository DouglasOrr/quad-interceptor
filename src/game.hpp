#pragma once

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <vector>

#include "maths.hpp"

// Always-on assertion; overrides the standard `assert` so it stays active in
// release builds (which define NDEBUG).
[[noreturn]] inline void gg_assert_fail(const char* expr, const char* file, int line) {
    std::fprintf(stderr, "assertion failed: %s (%s:%d)\n", expr, file, line);
    std::abort();
}
#define gassert(cond) ((cond) ? (void)0 : gg_assert_fail(#cond, __FILE__, __LINE__))

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
    enum class Status { Alive, Crashed, Detonated };

    inline static constexpr float height = 0.15f;
    inline static constexpr float radius = 0.45f;
    inline static constexpr std::array<Vec<3>, 4> motor_offsets_m = {{
        {{0.1f, 0.1f, 0.0f}},    // FL
        {{0.1f, -0.1f, 0.0f}},   // FR
        {{-0.1f, 0.1f, 0.0f}},   // BL
        {{-0.1f, -0.1f, 0.0f}},  // BR
    }};

    // State
    Vec<3> position;      // (world)
    Vec<3> velocity;      // (world)
    Quat orientation;     // (model -> world)
    Vec<3> ang_momentum;  // (world)
    Status status = Status::Alive;

    // Control
    Vec<4> control_m;    // (roll R/L, pitch F/B, vertical U/D, yaw CW/CCW) in [-1, 1]
    Vec<4> motor_speed;  // (FL, FR, BL, BR) in [0, 1]
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
    inline static constexpr uint block_size = 10;  // cells per city block (street grid period)
    inline static constexpr uint street_size = 2;  // street width, in cells (each block edge)

    int64 seed;
    Cell operator()(int64 x, int64 y) const;  // (x, y) are cell coordinates
};

struct GameState {
    uint64 tick;
    Vec<3> spawn;               // player respawn position
    uint64 respawn_time;        // physics tick at which the player respawns (0 = none)
    std::vector<Drone> drones;  // player = drones[0]
    City city;
    Vec<3> camera_pos;
    Quat camera_orientation;
};

///////////////////////////////////////////////////////////////////////////////
// Game Manager

struct GameManager {
    void update(GameState& state);
};

///////////////////////////////////////////////////////////////////////////////
// Physics

struct Physics {
    inline static constexpr float dt = 1e-3f;  // seconds

    static void tick(GameState& state);
};

///////////////////////////////////////////////////////////////////////////////
// Assets

struct Font {
    struct Glyph {
        uint8_t ch;
        std::vector<uint8_t> data;  // width*height bytes, each 0 or 255
    };
    uint width;
    uint height;
    std::vector<Glyph> glyphs;

    static Font load();
};

///////////////////////////////////////////////////////////////////////////////
// Renderer

struct Renderer {
    Renderer();
    ~Renderer();

    void draw(const GameState& state, float dt);

private:
    struct State;
    std::unique_ptr<State> state;
};

///////////////////////////////////////////////////////////////////////////////
// Audio

struct AudioEngine {
    AudioEngine();
    ~AudioEngine();

    void update(const GameState& state);
    void fill_buffer(int16_t* stream, int num_samples);

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
