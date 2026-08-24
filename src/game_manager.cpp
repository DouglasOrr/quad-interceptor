#include <utility>
#include <vector>

#include "game.hpp"
#include "platform.hpp"

////////////////////////////////////////////////////////////////////////////////
// Assets

// Serialized font blob, defined in the generated font_data.cpp.
std::span<const uint8_t> font_bytes();

Font Font::load() {
    Font font;
    auto data = font_bytes();
    const uint8_t n = data[0];
    font.width = data[1];
    font.height = data[2];
    const size_t n_pixels = font.width * font.height;
    gassert(n_pixels % 8 == 0);
    const size_t cell_bytes = n_pixels / 8;
    gassert(data.size() == 3 + n * (1 + cell_bytes));
    font.glyphs.reserve(n);
    for (uint8_t i = 0; i < n; ++i) {
        const size_t off = 3 + size_t(i) * (1 + cell_bytes);
        std::vector<uint8_t> px(n_pixels);
        for (size_t p = 0; p < n_pixels; ++p)
            px[p] = (data[off + 1 + p / 8] >> (7 - p % 8)) & 1 ? 255 : 0;
        font.glyphs.push_back({data[off], std::move(px)});
    }
    return font;
}

////////////////////////////////////////////////////////////////////////////////
// GameManager

namespace {
constexpr float city_blocks(float n) {
    return n * float(City::block_size) * City::cell_size;
}

constexpr float respawn_delay = 2.0f;
constexpr float player_spawn_alt = 5.0f;
constexpr float enemy_spawn_alt = 20.0f;
constexpr float sandbox_kill_radius = city_blocks(6);
constexpr float chase_escape_radius = city_blocks(2.5);
constexpr Vec<3> cam_offset_vec{-1.2f, 0.0f, 0.5f};  // chase (behind, slightly up)
constexpr float cam_horizon_bias = 0.2f;             // 0 = none; 1 = look fully at z=0 horizon

Vec<3> player_spawn(const GameState& s, const std::optional<Drone>& player) {
    Vec<2> c;
    if (s.mission.type == Mission::Chase && player) {
        c = City::intersection_center(
            City::nearest_intersection_block({player->position[0], player->position[1]}));
    } else if (s.mission.objectives.empty()) {
        c = City::intersection_center(City::nearest_intersection_block({0.0f, 0.0f}));
    } else {
        c = s.mission.objectives[std::max(s.next_objective, 1u) - 1].center;
    }
    return {c[0], c[1], player_spawn_alt};
}

Drone& spawn_drone(uint& next_drone_id,
                   GameState& s,
                   Drone::Kind kind,
                   const DroneType& type,
                   Vec<3> position) {
    if (kind == Drone::Kind::Player) {
        std::erase_if(s.drones, [](const Drone& d) { return d.kind == Drone::Kind::Player; });
    }
    Drone d;
    d.id = next_drone_id++;
    d.kind = kind;
    d.type = type;
    d.position = position;
    d.velocity = {};
    d.orientation = {0.0f, 0.0f, 0.0f, 1.0f};
    d.ang_momentum = {};
    d.status = Drone::Status::Alive;
    d.lock_on = 0;
    d.control_m = {};
    d.motor_speed = {};
    s.drones.push_back(d);
    return s.drones.back();
}
}  // namespace

std::vector<DroneType> DroneType::all() {
    DroneType acro;
    acro.name = "acro";
    acro.mass = 1.0f;
    acro.inv_inertia_m = mat_diag(0.4f, 0.4f, 1.5f);
    acro.damping_m = mat_diag(0.2f, 0.2f, 1.0f);
    acro.ang_damping_m = mat_diag(0.4f, 0.4f, 0.4f);
    acro.motor_force = 8.0f;
    acro.motor_torque = 0.6f;
    acro.motor_offsets_m = DroneType::motor_offsets_from_radius(0.5f);
    acro.height = 0.15f;
    acro.collision_radius = 0.4f;
    acro.control = Control::Acro;
    Pid rate = {.target = 2.5f, .lo = -1.0f, .hi = 1.0f, .kp = 5.0f, .ki = 3.0f, .kd = 0.0f};
    acro.pid_roll_rate = rate;
    acro.pid_pitch_rate = rate;
    acro.pid_yaw_rate = rate;

    DroneType tilt = acro;
    tilt.name = "tilt";
    tilt.control = Control::Tilt;
    tilt.damping_m = mat_diag(0.2f, 2.0f, 1.0f);  // high lateral drag, so it doesn't slide sideways
    Pid angle = {
        .target = 35.0f * deg_to_rad, .lo = -1.0f, .hi = 1.0f, .kp = 3.0f, .ki = 0.0f, .kd = 0.2f};
    tilt.pid_roll_angle = angle;
    tilt.pid_pitch_angle = angle;

    return {acro, tilt};
}

DroneType DroneType::enemy() {
    DroneType type = all().front();  // identical to the player's acro drone (for now)
    type.name = "enemy";
    return type;
}

std::vector<Mission> Mission::all() {
    DroneType default_player = DroneType::all().front();
    std::vector<Mission> missions;

    Mission sandbox;
    sandbox.type = Mission::Sandbox;
    sandbox.name = "sandbox";
    sandbox.player_type = default_player;
    sandbox.enemy_spawn_radius = city_blocks(2);
    sandbox.enemy_spawn_interval = 10.0f;
    sandbox.max_enemies = 3;
    sandbox.starting_lives = 0;
    sandbox.time_limit = 0.0f;
    sandbox.player_ai = false;
    missions.push_back(sandbox);

    Mission defense;
    defense.type = Mission::Defense;
    defense.name = "defense";
    defense.player_type = default_player;
    defense.enemy_spawn_radius = city_blocks(3);
    defense.enemy_spawn_interval = 10.0f;
    defense.max_enemies = 3;
    defense.starting_lives = 5;
    defense.time_limit = 0.0f;
    defense.player_ai = false;
    defense.objectives = {Circle{
        City::intersection_center(City::nearest_intersection_block({0, 0})), City::cell_size}};
    missions.push_back(defense);

    Mission race;
    race.type = Mission::Race;
    race.name = "race";
    race.player_type = default_player;
    race.enemy_spawn_radius = 0.0f;
    race.enemy_spawn_interval = 0.0f;
    race.max_enemies = 0;
    race.starting_lives = 0;
    race.time_limit = 0.0f;
    race.player_ai = false;
    const std::vector<City::Pos> waypoints = {{0, 0},  {0, 2},   {2, 2},  {2, 0}, {2, -2},
                                              {0, -2}, {-2, -2}, {-2, 0}, {0, 0}};
    for (auto p : waypoints) {
        race.objectives.push_back({City::intersection_center(p), City::cell_size});
    }
    missions.push_back(race);

    Mission chase;
    chase.type = Mission::Chase;
    chase.name = "chase";
    chase.player_type = default_player;
    chase.enemy_spawn_radius = city_blocks(1);
    chase.enemy_spawn_interval = 0.0f;
    chase.max_enemies = 1;
    chase.starting_lives = 3;
    chase.time_limit = 60.0f;
    chase.player_ai = false;
    missions.push_back(chase);

    return missions;
}

struct GameManager::State {
    uint next_drone_id = 0;
    uint64 player_respawn_tick = 0;
    uint64 next_enemy_spawn_tick = 0;
    uint64 race_start_tick = 0;
    bool player_was_alive = true;
};

GameManager::GameManager() : state(std::make_unique<State>()) {}
GameManager::~GameManager() = default;

void GameManager::update(GameState& s) {
    // Ensure a player exists (initial spawn / after a dead player is cleaned up).
    bool has_player = std::any_of(s.drones.begin(), s.drones.end(),
                                  [](auto& d) { return d.kind == Drone::Kind::Player; });
    if (!has_player)
        spawn_drone(state->next_drone_id, s, Drone::Kind::Player, s.mission.player_type,
                    player_spawn(s, std::nullopt));
    auto& player = player_drone(s);

    if (s.mission.time_limit > 0 && s.mission.type != Mission::Race) {
        auto end_tick = uint64(s.mission.time_limit / Physics::dt);
        s.timer = (s.tick < end_tick) ? (end_tick - s.tick) : 0;
    }

    // Mission updates
    if (s.mission.type == Mission::Defense) {
        for (auto& d : s.drones) {
            if (d.kind == Drone::Kind::Enemy && d.status == Drone::Status::Alive) {
                for (const auto& o : s.mission.objectives) {
                    if (length(Vec<2>{d.position[0], d.position[1]} - o.center) <= o.radius) {
                        d.status = Drone::Status::Detonated;
                        if (s.lives > 0) --s.lives;
                        break;
                    }
                }
            }
        }
    }
    if (s.mission.type == Mission::Race) {
        auto finished = s.next_objective >= s.mission.objectives.size();
        if ((player.status == Drone::Status::Alive) && !finished) {
            Vec<2> p_xy = {player.position[0], player.position[1]};
            if (!s.timer) {
                if (length(p_xy - s.mission.objectives[0].center) >
                    s.mission.objectives[0].radius) {
                    state->race_start_tick = s.tick;
                    s.timer = 0;
                    s.next_objective = 1;
                }
            } else {
                if (length(p_xy - s.mission.objectives[s.next_objective].center) <=
                    s.mission.objectives[s.next_objective].radius) {
                    ++s.next_objective;
                }
            }
        }
        if (s.timer && !finished) {
            s.timer = s.tick - state->race_start_tick;
        }
    }
    if (s.mission.type == Mission::Chase) {
        if (player.status == Drone::Status::Detonated && state->player_was_alive) {
            ++s.kill_count;
        }
    }
    state->player_was_alive = (player.status == Drone::Status::Alive);

    // Player respawn
    if (player.status == Drone::Status::Alive) {
        state->player_respawn_tick = 0;
    } else {
        if (state->player_respawn_tick == 0)
            state->player_respawn_tick = s.tick + uint64(respawn_delay / Physics::dt);
        if (s.tick >= state->player_respawn_tick) {
            state->player_respawn_tick = 0;
            spawn_drone(state->next_drone_id, s, Drone::Kind::Player, s.mission.player_type,
                        player_spawn(s, player));
        }
    }

    // Enemy cleanup: dead drones, and (sandbox) drones strayed beyond the kill radius.
    std::erase_if(s.drones, [&](const Drone& d) {
        if (d.kind != Drone::Kind::Enemy) return false;
        if (d.status != Drone::Status::Alive) return true;
        if (s.mission.type == Mission::Sandbox) {
            Vec<2> rel = {d.position[0] - player.position[0], d.position[1] - player.position[1]};
            return length(rel) > sandbox_kill_radius;
        }
        if (s.mission.type == Mission::Chase) {
            Vec<2> rel = {d.position[0] - player.position[0], d.position[1] - player.position[1]};
            if (length(rel) > chase_escape_radius) {
                if (s.lives > 0) --s.lives;
                return true;
            }
        }
        return false;
    });

    // Enemy spawn
    if (state->next_enemy_spawn_tick == 0) {
        state->next_enemy_spawn_tick =
            s.tick + uint64(s.mission.enemy_spawn_interval / Physics::dt);
    } else if (s.tick >= state->next_enemy_spawn_tick) {
        size_t enemy_count = 0;
        foreach_drone(s, [&](size_t, const Drone& d) {
            if (d.kind == Drone::Kind::Enemy) ++enemy_count;
        });
        if (enemy_count < s.mission.max_enemies) {
            uint64 h = hash(s.city.seed, s.tick, state->next_drone_id);
            float angle = (h & 0xffff) * (2.0f * pi / float(1 << 16));
            Vec<2> center = s.mission.objectives.empty()
                                ? Vec<2>{player.position[0], player.position[1]}
                                : s.mission.objectives[0].center;
            Vec<2> target_pos = {center[0] + s.mission.enemy_spawn_radius * std::cos(angle),
                                 center[1] + s.mission.enemy_spawn_radius * std::sin(angle)};
            Vec<2> spawn_xy =
                City::intersection_center(City::nearest_intersection_block(target_pos));
            spawn_drone(state->next_drone_id, s, Drone::Kind::Enemy, DroneType::enemy(),
                        {spawn_xy[0], spawn_xy[1], enemy_spawn_alt});
            state->next_enemy_spawn_tick = 0;
        }
    }

    // Player camera update
    const auto& p = player_drone(s);
    s.camera_pos = p.position + rotate(p.orientation, cam_offset_vec);
    // Bias the view towards the z=0 horizon: flatten the camera's pitch by a
    // fraction (preserving heading and roll) so the horizon is better framed.
    Vec<3> fwd = rotate(p.orientation, Vec<3>{1.0f, 0.0f, 0.0f});
    float pitch = -std::asin(std::clamp(fwd[2] / length(fwd), -1.0f, 1.0f));
    Vec<3> right = rotate(p.orientation, Vec<3>{0.0f, -1.0f, 0.0f});
    Quat bias = quat_from_axis_angle(right, cam_horizon_bias * pitch);
    s.camera_orientation = normalise(bias * p.orientation);
}

////////////////////////////////////////////////////////////////////////////////
// game_loop

namespace {
void copy_control(const Window& w, Vec<4>& c) {
    auto f = [](bool b) { return b ? 1.0f : 0.0f; };
    c = {};
    // Arrows
    c[0] += f(w.held(Key::right)) - f(w.held(Key::left));
    c[1] += f(w.held(Key::up)) - f(w.held(Key::down));
    // ZXC
    c[2] += 2 * f(w.held(Key::x)) - 1;
    c[3] += f(w.held(Key::c)) - f(w.held(Key::z));
}
}  // namespace

void game_loop(Window& window, const Mission& mission) {
    GameState state{};
    state.mission = mission;
    state.lives = state.mission.starting_lives;
    GameManager game_manager;
    game_manager.update(state);
    AI ai;
    Physics physics;
    Renderer renderer;
    AudioEngine audio_engine;
    window.set_audio_callback([&audio_engine](int16_t* stream, int num_frames) {
        audio_engine.fill_buffer(stream, num_frames);
    });
    Stopwatch stopwatch;
    float physics_time = 0.0f;
    while (window.frame()) {
        const float dt = stopwatch.lap();
        if (mission.time_limit > 0 && state.tick * Physics::dt >= mission.time_limit) break;
        if (window.pressed(Key::hash)) {
            game_loop(window, mission);
            return;
        }
        if (!mission.player_ai) copy_control(window, player_drone(state).control_m);
        physics_time = std::min(0.25f, physics_time + dt);
        while (physics_time >= Physics::dt) {
            ai.update(state);
            physics.tick(state);
            physics_time -= Physics::dt;
        }
        game_manager.update(state);
        renderer.draw(state, dt);
        audio_engine.update(state);
    }
    window.set_audio_callback({});  // the window's callback can still call into the audio_engine
}
