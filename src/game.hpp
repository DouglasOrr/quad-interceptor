#pragma once

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "maths.hpp"

struct Window;  // defined in platform.hpp

struct Circle {
    Vec<2> center;
    float radius;
};

////////////////////////////////////////////////////////////////////////////////
// State
//
// Default world frame; an '_m' suffix marks a model(body)-frame quantity.

struct Pid {
    float target;
    float lo, hi;
    float kp, ki, kd;
};

struct DroneType {
    std::string name;

    // Airframe
    float mass;
    Mat3 inv_inertia_m;
    Mat3 damping_m;      // linear drag
    Mat3 ang_damping_m;  // angular drag
    float motor_force;   // thrust per unit motor command
    float motor_torque;  // yaw reaction torque per unit motor command

    // Geometry
    std::array<Vec<3>, 4> motor_offsets_m;  // (FL, FR, BL, BR)
    float height;
    float collision_radius;

    // Control
    float mix_roll_to_yaw;
    std::optional<Pid> pid_roll_rate, pid_pitch_rate, pid_lift_rate, pid_yaw_rate;
    std::optional<Pid> pid_roll_angle, pid_pitch_angle;  // Tilt only; target = max tilt angle

    // Factories
    static std::array<Vec<3>, 4> motor_offsets_from_radius(float radius);
    static std::vector<DroneType> all();  // player-selectable variants
    static DroneType enemy();
};

struct Drone {
    enum class Status { Alive, Crashed, Detonated };
    enum class Kind { Player, Enemy };

    // Identity
    uint id;
    Kind kind;
    DroneType type;

    // State
    Vec<3> position;      // (world)
    Vec<3> velocity;      // (world)
    Quat orientation;     // (model -> world)
    Vec<3> ang_momentum;  // (world)
    Status status;
    float lock_on;

    // Control
    Vec<4> control_m;    // (roll R/L, pitch F/B, lift U/D, yaw CW/CCW) in [-1, 1]
    Vec<4> motor_speed;  // (FL, FR, BL, BR) in [0, 1]
};

struct TerrainType {
    enum class Kind : std::uint8_t { City, Plains };

    // City: blocks of towers, some with "stilt" gaps and skywalks.
    struct City {
        inline static constexpr float skywalk_thickness = 5.0f;
        inline static constexpr float pillar_radius = 1.2f;

        float lock_on_level;
        uint block_size;   // cells
        uint street_size;  // cells
        Vec<2> building_height_range;
        float stilt_prob;
        Vec<2> stilt_podium_range;
        Vec<2> stilt_gap_range;
        float skywalk_prob;
        Vec<2> skywalk_height_range;

        Vec<2> snap_intersection(Vec<2> v) const;
    };

    // Plains: open ground with scattered poles and a power-line pylon grid.
    struct Plains {
        inline static constexpr float pylon_radius = 1.3f;
        inline static constexpr float pylon_height = 45.0f;

        float pole_prob;
        Vec<2> pole_radius_range;
        Vec<2> pole_height_range;
        float pylon_run_prob;
        uint pylon_row_period;
        uint pylon_stride;
    };

    Kind kind;
    union {
        City city;
        Plains plains;
    };
};

struct Terrain {
    enum class CellType : std::uint8_t { Empty, Building, Skywalk, Pole };

    struct BuildingData {
        float top;     // tower [base, top]
        float base;    // stilts [podium, base]
        float podium;  // lower floors [0, podium]
    };
    struct SkywalkData {
        float bottom;
        bool axis_x;
    };
    struct PoleData {
        float radius;
        float top;
        bool pylon_x;  // horizontal power-line run
        bool pylon_y;  // vertical power-line run
    };

    struct Cell {
        CellType type;
        uint64 hash;
        union {
            struct {
            } empty;
            BuildingData building;
            SkywalkData skywalk;
            PoleData pole;
        };
    };
    using Pos = std::array<int64, 2>;

    int64 seed;
    TerrainType type;

    Cell operator()(Pos p) const;

    inline static constexpr float cell_size = 15.0f;
    static Pos nearest_cell(Vec<2> v);
    static Vec<2> cell_center(Pos p);
};

struct Mission {
    enum Type { Defense, Sandbox, Race, Chase };
    Type type;
    std::string name;
    TerrainType terrain_type;
    DroneType player_type;
    std::vector<Circle> objectives;  // empty in sandbox
    float enemy_spawn_radius;        // around objective (defense) or player (sandbox, chase)
    float enemy_spawn_interval;
    uint max_enemies;
    uint starting_lives;
    float time_limit;
    bool player_ai;

    static std::vector<Mission> all();
};

struct GameState {
    uint64 tick;
    std::vector<Drone> drones;
    Vec<3> camera_pos;
    Quat camera_orientation;
    Terrain terrain;
    Mission mission;
    // Mission state
    uint lives;
    uint kill_count;
    uint next_objective;
    std::optional<uint64> timer;
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

void game_loop(Window& window, const Mission& mission);

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

    Physics();
    ~Physics();

    void tick(GameState& state);

private:
    struct State;
    std::unique_ptr<State> state;
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
inline void print(const Args&... args) {
    ((std::cerr << ' ' << args), ...);
    std::cerr << '\n';
}

template <typename... Args>
inline void print(const GameState& state, uint period, const Args&... args) {
    if (state.tick % period == 0) {
        std::cerr << '[' << state.tick << "]";
        print(args...);
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
