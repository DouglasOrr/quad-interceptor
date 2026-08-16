#pragma once

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

#include "maths.hpp"

struct Window;  // defined in platform.hpp

////////////////////////////////////////////////////////////////////////////////
// State
//
// Default world frame; an '_m' suffix marks a model(body)-frame quantity.

struct Pid {
    // config
    float target;
    float lo, hi;
    float kp, ki, kd;
    bool enabled = true;
    // state
    float integral = 0.0f;
    float prev = 0.0f;
    float operator()(float control, float actual, float dt);
};

struct Drone {
    enum class Status { Alive, Crashed, Detonated };
    enum class Kind { Player, Enemy };

    inline static constexpr float height = 0.15f;
    inline static constexpr float collision_radius = 0.4f;
    inline static constexpr float _radius = 0.5f;
    inline static constexpr float lock_on_level = 80.0f;
    inline static constexpr std::array<Vec<3>, 4> motor_offsets_m = {{
        {{_radius / M_SQRT2f, _radius / M_SQRT2f, 0.0f}},    // FL
        {{_radius / M_SQRT2f, -_radius / M_SQRT2f, 0.0f}},   // FR
        {{-_radius / M_SQRT2f, _radius / M_SQRT2f, 0.0f}},   // BL
        {{-_radius / M_SQRT2f, -_radius / M_SQRT2f, 0.0f}},  // BR
    }};

    // Identity
    uint id;
    Kind kind;

    // State
    Vec<3> position;      // (world)
    Vec<3> velocity;      // (world)
    Quat orientation;     // (model -> world)
    Vec<3> ang_momentum;  // (world)
    Status status;
    float lock_on;

    // Control
    Vec<4> control_m;        // (roll R/L, pitch F/B, lift U/D, yaw CW/CCW) in [-1, 1]
    std::array<Pid, 4> pid;  // (roll, pitch, lift, yaw)
    Vec<4> motor_speed;      // (FL, FR, BL, BR) in [0, 1]
};

struct Cell {
    float height;  // 0=street
    uint64 hash;
};

struct City {
    using Pos = std::array<int64, 2>;
    inline static constexpr uint cell_size = 15;
    inline static constexpr uint block_size = 10;  // cells per city block (street grid period)
    inline static constexpr uint street_size = 2;  // street width, in cells (each block edge)

    int64 seed;
    Cell operator()(Pos p) const;
    static Pos nearest_cell(Vec<2> v);
    static Vec<2> cell_center(Pos p);
    static Pos nearest_intersection_block(Vec<2> v);
    static Vec<2> intersection_center(Pos block);
};

struct GameState {
    uint64 tick;
    std::vector<Drone> drones;
    City city;
    Vec<3> camera_pos;
    Quat camera_orientation;

    // Mission
    Vec<3> spawn;
    Vec<2> objective;
    float objective_radius;
    int lives;
};

///////////////////////////////////////////////////////////////////////////////
// Game Manager & Assets (game_manager.cpp)

struct GameManager {
    GameManager();
    ~GameManager();

    void update(GameState& state);

private:
    struct State;
    std::unique_ptr<State> state;
};

void game_loop(Window& window);

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
// Physics & AI (physics.cpp)

struct Physics {
    inline static constexpr float dt = 1e-3f;  // seconds

    static void tick(GameState& state);
};

struct AI {
    AI();
    ~AI();

    void update(GameState& state);

private:
    struct State;
    std::unique_ptr<State> state;
};

///////////////////////////////////////////////////////////////////////////////
// Renderer (renderer.cpp)

struct Renderer {
    Renderer();
    ~Renderer();

    void draw(const GameState& state, float dt);

private:
    struct State;
    std::unique_ptr<State> state;
};

///////////////////////////////////////////////////////////////////////////////
// Audio (audio.cpp)

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

// Always-on assertion; overrides the standard `assert` so it stays active in
// release builds (which define NDEBUG).
[[noreturn]] inline void gg_assert_fail(const char* expr, const char* file, int line) {
    std::fprintf(stderr, "assertion failed: %s (%s:%d)\n", expr, file, line);
    std::abort();
}
#define gassert(cond) ((cond) ? (void)0 : gg_assert_fail(#cond, __FILE__, __LINE__))

template <typename... Args>
inline void print(const GameState& state, uint period, const Args&... args) {
    if (state.tick % period == 0) {
        std::cerr << '[' << state.tick << "]";
        ((std::cerr << ' ' << args), ...);
        std::cerr << '\n';
    }
}

// Iterate drones, defaulting to alive drones
template <typename F>
void foreach_drone(GameState& state,
                   F&& f,
                   std::optional<Drone::Status> status = Drone::Status::Alive) {
    for (size_t i = 0; i < state.drones.size(); ++i) {
        if (!status || (state.drones[i].status == *status)) {
            f(i, state.drones[i]);
        }
    }
}
template <typename F>
void foreach_drone(const GameState& state,
                   F&& f,
                   std::optional<Drone::Status> status = Drone::Status::Alive) {
    for (size_t i = 0; i < state.drones.size(); ++i) {
        if (!status || (state.drones[i].status == *status)) {
            f(i, state.drones[i]);
        }
    }
}

// Synchronise a parallel vector of items that have a .id attribute and are sorted by id
template <typename Item, typename ListA, typename InitFn, typename Fn>
void sync_parallel(ListA& list_a, std::vector<Item>& list_b, InitFn&& init, Fn&& fn) {
    std::vector<Item> updated;
    updated.reserve(list_a.size());
    size_t b_idx = 0;
    for (auto& a : list_a) {
        while (b_idx < list_b.size() && list_b[b_idx].id < a.id) {
            ++b_idx;
        }
        if (b_idx < list_b.size() && list_b[b_idx].id == a.id) {
            updated.push_back(std::move(list_b[b_idx]));
            ++b_idx;
        } else {
            updated.push_back(init(a));
        }
        fn(a, updated.back());
    }
    list_b = std::move(updated);
}

inline Drone& player_drone(GameState& state) {
    for (auto& d : state.drones) {
        if (d.kind == Drone::Kind::Player) return d;
    }
    gassert(false);
}
inline const Drone& player_drone(const GameState& state) {
    for (const auto& d : state.drones) {
        if (d.kind == Drone::Kind::Player) return d;
    }
    gassert(false);
}
