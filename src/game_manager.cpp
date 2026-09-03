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
constexpr float respawn_delay_detonation = 1.0f;
constexpr float respawn_delay_crash = 3.0f;
constexpr float finish_delay = 2.0f;
constexpr float player_spawn_alt = 20.0f;
constexpr float sandbox_kill_radius = 60.0f * Terrain::cell_size;  // 6 city blocks
constexpr float chase_escape_radius = 25.0f * Terrain::cell_size;  // 2.5 city blocks
constexpr Vec<3> cam_offset_vec{-1.2f, 0.0f, 0.5f};                // chase (behind, slightly up)
constexpr float cam_horizon_bias = 0.2f;  // 0 = none; 1 = look fully at z=0 horizon

Vec<2> spawn_pos(const GameState& s, Vec<2> target) {
    if (s.terrain.type.kind == TerrainType::Kind::City) {
        return s.terrain.type.city.snap_intersection(target);
    }
    const float range = 3.0f;
    for (uint i = 0; i < 32; ++i) {  // rejection sample to find an empty cell
        uint64 h = hash(s.terrain.seed, s.tick, i);
        Vec<2> jitter = {hash_u(h, 0, -range, range), hash_u(h, 16, -range, range)};
        Vec<2> candidate = target + Terrain::cell_size * jitter;
        auto cell = s.terrain(Terrain::nearest_cell(candidate));
        if (cell.type == Terrain::CellType::Empty) return candidate;
    }
    return target;
}

Vec<3> player_spawn(const GameState& s, const std::optional<Drone>& player) {
    Vec<2> c;
    if (s.mission.type == Mission::Defense && player) {
        Vec<2> ref = Vec<2>{player->position[0], player->position[1]};
        c = s.mission.objectives[0].center;
        float best = length(ref - c);
        for (const auto& o : s.mission.objectives) {
            auto d = length(ref - o.center);
            if (d < best) {
                best = d;
                c = o.center;
            }
        }
    } else if (s.mission.type == Mission::Chase && player) {
        c = spawn_pos(s, {player->position[0], player->position[1]});
    } else if (!s.mission.objectives.empty()) {
        c = s.mission.objectives[std::max(s.next_objective, 1u) - 1].center;
    } else {
        c = spawn_pos(s, {0.0f, 0.0f});
    }
    return {c[0], c[1], player_spawn_alt};
}

Vec<3> enemy_spawn(const GameState& s, uint64 seed) {
    auto& objectives = s.mission.objectives;
    const float radius = s.mission.enemy_spawn_radius;
    const uint64 h0 = hash(seed, 0xa4a6d5a7, s.tick);
    const float alt = hash_u(h0, 16, s.mission.enemy_spawn_alt[0], s.mission.enemy_spawn_alt[1]);

    Vec<2> center;
    if (objectives.empty()) {
        auto& player = player_drone(s);
        center = {player.position[0], player.position[1]};
    } else {
        center = objectives[h0 % objectives.size()].center;
    }
    for (uint i = 0; true; ++i) {  // rejection sample - don't land directly on an objective
        uint64 h = hash(h0, uint64(i));
        Vec<2> p = spawn_pos(s, {center[0] + radius * std::cos(hash_u(h, 0, 0.0f, 2.0f * pi)),
                                 center[1] + radius * std::sin(hash_u(h, 16, 0.0f, 2.0f * pi))});
        auto on_objective = std::any_of(objectives.begin(), objectives.end(),
                                        [p](auto& o) { return length(p - o.center) <= o.radius; });
        if (!on_objective || i >= 32) {
            return {p[0], p[1], alt};
        }
    }
    gassert(false && "unreachable");
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
    std::vector<DroneType> drones;

    {
        DroneType tilt;
        tilt.name = "tilt";
        tilt.mass = 1.0f;
        tilt.inv_inertia_m = mat_diag(0.4f, 0.4f, 1.5f);
        tilt.damping_m =
            mat_diag(0.2f, 2.0f, 1.0f);  // high lateral drag, so it doesn't slide sideways
        tilt.ang_damping_m = mat_diag(0.4f, 0.4f, 0.4f);
        tilt.motor_force = 8.0f;
        tilt.motor_torque = 0.6f;
        tilt.motor_offsets_m = DroneType::motor_offsets_from_radius(0.5f);
        tilt.height = 0.15f;
        tilt.collision_radius = 0.4f;
        tilt.mix_roll_to_yaw = 0.4f;
        Pid rate = {.target = 2.5f, .lo = -1.0f, .hi = 1.0f, .kp = 5.0f, .ki = 2.0f, .kd = 2.0f};
        tilt.pid_roll_rate = rate;
        tilt.pid_pitch_rate = rate;
        tilt.pid_yaw_rate = {
            .target = 1.5f, .lo = -1.0f, .hi = 1.0f, .kp = 5.0f, .ki = 0.0f, .kd = 0.0f};
        Pid angle = {.target = 35.0f * deg_to_rad,
                     .lo = -1.0f,
                     .hi = 1.0f,
                     .kp = 10.0f,
                     .ki = 0.0f,
                     .kd = 0.2f};
        tilt.pid_roll_angle = angle;
        tilt.pid_pitch_angle = angle;
        tilt.pid_lift_rate = {
            .target = 20.0f, .lo = -1.0f, .hi = 1.0f, .kp = 20.0f, .ki = 2.0f, .kd = 0.0f};
        drones.push_back(tilt);
    }
    {
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
        acro.mix_roll_to_yaw = 0.0f;
        Pid rate = {.target = 2.5f, .lo = -1.0f, .hi = 1.0f, .kp = 5.0f, .ki = 3.0f, .kd = 0.0f};
        acro.pid_roll_rate = rate;
        acro.pid_pitch_rate = rate;
        acro.pid_yaw_rate = rate;
        drones.push_back(acro);
    }
    drones.push_back(enemy());

    return drones;
}

DroneType DroneType::enemy() {
    Pid rate = {.target = 5.0f, .lo = -1.0f, .hi = 1.0f, .kp = 5.0f, .ki = 5.0f, .kd = 0.0f};
    return {
        .name = "enemy",
        .mass = 1.0f,
        .inv_inertia_m = mat_diag(0.4f, 0.4f, 1.5f),
        .damping_m = mat_diag(0.2f, 0.2f, 1.0f),
        .ang_damping_m = mat_diag(0.4f, 0.4f, 0.4f),
        .motor_force = 8.0f,
        .motor_torque = 0.6f,
        .motor_offsets_m = DroneType::motor_offsets_from_radius(0.5f),
        .height = 0.15f,
        .collision_radius = 0.4f,
        .mix_roll_to_yaw = 0.0f,
        .pid_roll_rate = rate,
        .pid_pitch_rate = rate,
        .pid_lift_rate = {},
        .pid_yaw_rate = rate,
        .pid_roll_angle = {},
        .pid_pitch_angle = {},
    };
}

std::vector<Mission> Mission::all() {
    auto drones = DroneType::all();
    auto drone_tilt =
        std::find_if(drones.begin(), drones.end(), [](auto& d) { return d.name == "tilt"; });
    auto drone_acro =
        std::find_if(drones.begin(), drones.end(), [](auto& d) { return d.name == "acro"; });
    TerrainType terrain_plains{.kind = TerrainType::Kind::Plains,
                               .plains = {.pole_prob = 0.06f,
                                          .pole_radius_range = {0.25f, 0.8f},
                                          .pole_height_range = {4.0f, 14.0f},
                                          .pylon_run_prob = 1.0f,
                                          .pylon_row_period = 48,
                                          .pylon_stride = 4}};
    TerrainType terrain_city{.kind = TerrainType::Kind::City,
                             .city = {.lock_on_level = 80.0f,
                                      .block_size = 10,
                                      .street_size = 2,
                                      .building_height_range = {10.0f, 100.0f},
                                      .stilt_prob = 0.5f,
                                      .stilt_podium_range = {1.0f, 40.0f},
                                      .stilt_gap_range = {5.0f, 20.0f},
                                      .skywalk_prob = 0.5f,
                                      .skywalk_height_range = {10.0f, 80.0f}}};
    auto C = Terrain::cell_size;
    auto B = terrain_city.city.block_size;
    std::vector<Mission> missions;

    // Tilt ----------------------

    missions.push_back({
        .type = Mission::Tutorial,
        .name = "Tutorial",
        .info = "Learn to fly (tilt mode) -- see the pause menu (ESC or top-left) for controls."
                " Try to fly to the green objective marker. You'll spot a red dot on your minimap."
                " That's an enemy drone. Fly close to it to detonate yourself (and it)."
                " To finish the mission, fly through all green markers and destroy an enemy drone.",
        .seed = 0xbf4f139e,
        .terrain_type = terrain_plains,
        .player_type = *drone_tilt,
        .objectives =
            {
                {Vec<2>{0.0f, 0.0f}, C},
                {Vec<2>{100.0f, 0.0f}, C},
                {Vec<2>{-100.0f, 300.0f}, C},
                {Vec<2>{-100.0f, -300.0f}, C},
            },
        .max_enemies = 1,
        .enemy_spawn_radius = 4.0f * C,
        .enemy_spawn_alt = {18.0f, 22.0f},
        .enemy_spawn_interval = 20.0f,
        .enemy_tilt = 0.0f,
        .starting_lives = 0,
        .target_time = 0,
        .kill_bonus_time = 0,
        .target_kill_count = 1,
    });

    missions.push_back({
        .type = Mission::Race,
        .name = "Race Across the Plains",
        .info = "Complete the course as fast as possible.",
        .seed = 0xc6157a1c,
        .terrain_type = terrain_plains,
        .player_type = *drone_tilt,
        .objectives =
            {
                {Vec<2>{0.0f, -50.0f}, C},
                {Vec<2>{100.0f, -50.0f}, C},
                {Vec<2>{150.0f, -150.0f}, C},
                {Vec<2>{100.0f, -250.0f}, C},
                {Vec<2>{0.0f, -100.0f}, C},
                {Vec<2>{-25.0f, 10.0f}, C / 2},
                {Vec<2>{-80.0f, -5.0f}, C / 2},
                {Vec<2>{-120.0f, 10.0f}, C / 2},
                {Vec<2>{-160.0f, -5.0f}, C / 2},
                {Vec<2>{-200.0f, 5.0f}, C / 2},
                {Vec<2>{-240.0f, -5.0f}, C / 2},
                {Vec<2>{-0.0f, 40.0f}, C},
            },
        .max_enemies = 0,
        .enemy_spawn_radius = 0,
        .enemy_spawn_alt = {0.0f, 0.0f},
        .enemy_spawn_interval = 0,
        .enemy_tilt = 0,
        .starting_lives = 0,
        .target_time = 60.0f,
        .kill_bonus_time = 0,
        .target_kill_count = 0,
    });

    missions.push_back({
        .type = Mission::Chase,
        .name = "Hunting 101",
        .info = "Destroy 5 enemies before you run out of time.",
        .seed = 0x9c58daec,
        .terrain_type = terrain_plains,
        .player_type = *drone_tilt,
        .objectives = {},
        .max_enemies = 1,
        .enemy_spawn_radius = 80.0f,
        .enemy_spawn_alt = {15.0f, 30.0f},
        .enemy_spawn_interval = 1.0f,
        .enemy_tilt = 0.5f,
        .starting_lives = 0,
        .target_time = 45.0f,
        .kill_bonus_time = 10.0f,
        .target_kill_count = 5,
    });

    // Acro ----------------------

    missions.push_back({
        .type = Mission::Tutorial,
        .name = "Acro Tutorial",
        .info = "Learn to fly (acro mode) -- see the pause menu (ESC or top-left) for controls."
                " Acro mode is harder, but a lot more fun. You control the rate of change rather"
                " than the tilt angle. To finish the mission, fly through all green markers and"
                " destroy 3 enemies.",
        .seed = 0xcc24d21f,
        .terrain_type = terrain_plains,
        .player_type = *drone_acro,
        .objectives =
            {
                {Vec<2>{0.0f, 0.0f}, C},
                {Vec<2>{100.0f, 0.0f}, C},
                {Vec<2>{100.0f, 100.0f}, C},
                {Vec<2>{0.0f, 100.0f}, C},
                {Vec<2>{0.0f, -100.0f}, C},
            },
        .max_enemies = 3,
        .enemy_spawn_radius = 3.0f * C,
        .enemy_spawn_alt = {18.0f, 22.0f},
        .enemy_spawn_interval = 10.0f,
        .enemy_tilt = 0.0f,
        .starting_lives = 0,
        .target_time = 0,
        .kill_bonus_time = 0,
        .target_kill_count = 3,
    });

    missions.push_back({
        .type = Mission::Chase,
        .name = "Acrobatic Hunting",
        .info = "Destroy 5 enemies before you run out of time.",
        .seed = 0x54c6dfb1,
        .terrain_type = terrain_plains,
        .player_type = *drone_acro,
        .objectives = {},
        .max_enemies = 1,
        .enemy_spawn_radius = 80.0f,
        .enemy_spawn_alt = {15.0f, 30.0f},
        .enemy_spawn_interval = 1.0f,
        .enemy_tilt = 1.0f,
        .starting_lives = 0,
        .target_time = 60.0f,
        .kill_bonus_time = 15.0f,
        .target_kill_count = 5,
    });

    // City ----------------------

    auto city_circuit = [&](const std::vector<std::tuple<float, float>>& blocks) {
        std::vector<Circle> objectives;
        for (auto [bx, by] : blocks) {
            objectives.push_back(
                {terrain_city.city.snap_intersection({bx * B * C, by * B * C}), C});
        }
        return objectives;
    };

    missions.push_back({
        .type = Mission::Race,
        .name = "City Racer",
        .info = "Complete the course as fast as possible. Note that cities have an altitude limit;"
                " above this altitude, an enemy signal jammer will eventually lock on to you, and"
                " your connection will be lost.",
        .seed = 0xf80a9b64,
        .terrain_type = terrain_city,
        .player_type = *drone_acro,
        .objectives = city_circuit({
            {0, 0},
            {1, 0},
            {2, 0},
            {2, -1},
            {2, -2},
            {3, -2},
            {4, -3},
        }),
        .max_enemies = 0,
        .enemy_spawn_radius = 0,
        .enemy_spawn_alt = {0.0f, 0.0f},
        .enemy_spawn_interval = 0,
        .enemy_tilt = 0,
        .starting_lives = 0,
        .target_time = 120.0f,
        .kill_bonus_time = 0,
        .target_kill_count = 0,
    });

    missions.push_back({
        .type = Mission::Chase,
        .name = "Urban Fox",
        .info = "Destroy 5 enemies before you run out of time.",
        .seed = 0xbace38d1,
        .terrain_type = terrain_city,
        .player_type = *drone_acro,
        .objectives = {},
        .max_enemies = 3,
        .enemy_spawn_radius = B * C,
        .enemy_spawn_alt = {5.0f, 80.0f},
        .enemy_spawn_interval = 5.0f,
        .enemy_tilt = 0.8f,
        .starting_lives = 0,
        .target_time = 60.0f,
        .kill_bonus_time = 20.0f,
        .target_kill_count = 5,
    });

    missions.push_back({
        .type = Mission::Defense,
        .name = "Defender of the State",
        .info = "Stop enemies from reaching the objectives.",
        .seed = 0x04d4dd8d,
        .terrain_type = terrain_city,
        .player_type = *drone_acro,
        .objectives = city_circuit({{0, 0}, {1, 0}}),
        .max_enemies = 2,
        .enemy_spawn_radius = 2 * B * C,
        .enemy_spawn_alt = {15.0f, 40.0f},
        .enemy_spawn_interval = 10.0f,
        .enemy_tilt = 0.75f,
        .starting_lives = 3,
        .target_time = 180.0f,
        .kill_bonus_time = 0,
        .target_kill_count = 0,
    });

    missions.push_back({
        .type = Mission::Race,
        .name = "City Slicker",
        .info = "Complete the course as fast as possible.",
        .seed = 0x13fcea1e,
        .terrain_type = terrain_city,
        .player_type = *drone_acro,
        .objectives = city_circuit({
            {0, 0},
            {1, 0},
            {1, 1},
            {0, 2},
            {-2, 2},
            {-3, 1},
            {-2, 0},
            {-1, 0},
            {-1, -1},
            {0, -1},
            {0, -2},
            {1, -3},
            {4, -3},
            {3, -2},
        }),
        .max_enemies = 0,
        .enemy_spawn_radius = 0,
        .enemy_spawn_alt = {0.0f, 0.0f},
        .enemy_spawn_interval = 0,
        .enemy_tilt = 0,
        .starting_lives = 0,
        .target_time = 180.0f,
        .kill_bonus_time = 0,
        .target_kill_count = 0,
    });

    // Sandbox ----------------------

    missions.push_back({
        .type = Mission::Sandbox,
        .name = "Plains Sandbox",
        .info = "Free flight over the open plains. No objectives, no time limit."
                " Enemy drones present to keep you company.",
        .seed = 0x8a1ab115,
        .terrain_type = terrain_plains,
        .player_type = *drone_acro,
        .objectives = {},
        .max_enemies = 5,
        .enemy_spawn_radius = 10.0f * C,
        .enemy_spawn_alt = {15.0f, 30.0f},
        .enemy_spawn_interval = 15.0f,
        .enemy_tilt = 0.8f,
        .starting_lives = 0,
        .target_time = 0,
        .kill_bonus_time = 0,
        .target_kill_count = 0,
    });

    missions.push_back({
        .type = Mission::Sandbox,
        .name = "City Sandbox",
        .info = "Free flight in the infinite city. No objectives, no time limit."
                " Enemy drones present to keep you company.",
        .seed = 0xd6134a2b,
        .terrain_type = terrain_city,
        .player_type = *drone_acro,
        .objectives = {},
        .max_enemies = 5,
        .enemy_spawn_radius = 3.0f * B * C,
        .enemy_spawn_alt = {5.0f, 60.0f},
        .enemy_spawn_interval = 15.0f,
        .enemy_tilt = 0.8f,
        .starting_lives = 0,
        .target_time = 0,
        .kill_bonus_time = 0,
        .target_kill_count = 0,
    });

    return missions;
}

struct GameManager::State {
    uint next_drone_id = 0;
    uint64 player_respawn_tick = 0;
    uint64 next_enemy_spawn_tick = 0;
    uint64 race_start_tick = 0;
    uint64 countdown_prev_tick = 0;
};

GameManager::GameManager() : state(std::make_unique<State>()) {}
GameManager::~GameManager() = default;

void GameManager::reset() {
    *state = State{};
}

void GameManager::update(GameState& s) {
    // Ensure a player exists (initial spawn / after a dead player is cleaned up).
    bool has_player = std::any_of(s.drones.begin(), s.drones.end(),
                                  [](auto& d) { return d.kind == Drone::Kind::Player; });
    if (!has_player)
        spawn_drone(state->next_drone_id, s, Drone::Kind::Player, s.mission.player_type,
                    player_spawn(s, std::nullopt));
    auto& player = player_drone(s);

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
        if (s.result.type == MissionResult::None) {
            if (!s.timer.has_value()) {
                s.timer = uint64(s.mission.target_time / Physics::dt);
            } else {
                auto elapsed = s.tick - state->countdown_prev_tick;
                if (*s.timer > 0) *s.timer = *s.timer > elapsed ? *s.timer - elapsed : 0;
            }
            state->countdown_prev_tick = s.tick;
            const float time = float(s.timer.value_or(0)) * Physics::dt;
            if (*s.timer == 0) {
                s.result = {MissionResult::Completed, time, s.kill_count};
            } else if (s.lives == 0) {
                s.result = {MissionResult::Failed, time, s.kill_count};
            }
        }
    }
    if (s.mission.type == Mission::Chase && s.result.type == MissionResult::None) {
        if (!s.timer.has_value()) {
            s.timer = uint64(s.mission.target_time / Physics::dt);
        } else {
            auto elapsed = s.tick - state->countdown_prev_tick;
            if (*s.timer > 0) *s.timer = *s.timer > elapsed ? *s.timer - elapsed : 0;
        }
        state->countdown_prev_tick = s.tick;
        if (!s.finish_tick.has_value()) {
            const float time = float(s.timer.value_or(0)) * Physics::dt;
            if (s.kill_count >= s.mission.target_kill_count) {
                s.result = {MissionResult::Completed, time, s.kill_count};
            } else if (*s.timer == 0) {
                s.result = {MissionResult::Failed, time, s.kill_count};
            }
        }
    }
    if (s.mission.type == Mission::Race || s.mission.type == Mission::Tutorial) {
        auto finished = s.next_objective >= s.mission.objectives.size();
        if (s.mission.type == Mission::Race) {
            if (!s.timer.has_value()) {
                state->race_start_tick = s.tick;
                s.timer = 0;
            }
            if (!finished) {
                s.timer = s.tick - state->race_start_tick;
            }
        }
        if ((player.status == Drone::Status::Alive) && !finished) {
            Vec<2> p_xy = {player.position[0], player.position[1]};
            auto next = s.mission.objectives[s.next_objective];
            if (length(p_xy - next.center) <= next.radius) {
                ++s.next_objective;
            }
        }
        if (finished && !s.finish_tick.has_value()) {
            if (s.mission.type == Mission::Race || (s.mission.type == Mission::Tutorial &&
                                                    s.kill_count >= s.mission.target_kill_count)) {
                const float time = float(s.timer.value_or(0)) * Physics::dt;
                s.result = {MissionResult::Completed, time, s.kill_count};
                if (s.mission.type == Mission::Race && time > s.mission.target_time)
                    s.result.type = MissionResult::Failed;
            }
        }
    }
    if (!s.finish_tick.has_value() && s.result.type != MissionResult::None) {
        s.finish_tick = s.tick + uint64(finish_delay / Physics::dt);
    }

    // Player respawn
    if (player.status == Drone::Status::Alive) {
        state->player_respawn_tick = 0;
    } else {
        const float delay = (player.status == Drone::Status::Detonated) ? respawn_delay_detonation
                                                                        : respawn_delay_crash;
        if (state->player_respawn_tick == 0)
            state->player_respawn_tick = s.tick + uint64(delay / Physics::dt);
        if (s.tick >= state->player_respawn_tick) {
            state->player_respawn_tick = 0;
            spawn_drone(state->next_drone_id, s, Drone::Kind::Player, s.mission.player_type,
                        player_spawn(s, player));
        }
    }

    // Enemy cleanup: dead drones, and (sandbox) drones strayed beyond the kill radius.
    std::erase_if(s.drones, [&](const Drone& d) {
        if (d.kind != Drone::Kind::Enemy) return false;
        if (d.status != Drone::Status::Alive) {
            if (d.status == Drone::Status::Detonated) {
                ++s.kill_count;
                if (s.mission.type == Mission::Chase && s.timer)
                    *s.timer += uint64(s.mission.kill_bonus_time / Physics::dt);
            }
            return true;
        }
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
    if (s.tick >= state->next_enemy_spawn_tick) {
        size_t enemy_count = 0;
        foreach_drone(s, [&](size_t, const Drone& d) {
            if (d.kind == Drone::Kind::Enemy) ++enemy_count;
        });
        if (enemy_count < s.mission.max_enemies) {
            uint64 h = hash(s.terrain.seed, s.tick, state->next_drone_id);
            spawn_drone(state->next_drone_id, s, Drone::Kind::Enemy, DroneType::enemy(),
                        enemy_spawn(s, h));
            state->next_enemy_spawn_tick =
                s.tick + uint64(s.mission.enemy_spawn_interval / Physics::dt);
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
void copy_control(const Window& w, const Settings& settings, Vec<4>& c) {
    auto f = [](bool b) { return b ? 1.0f : 0.0f; };
    auto read = [&](Action::Type a) { return f(w.scancode_held(settings.bindings[size_t(a)])); };
    c = {};
    c[0] += read(Action::RollRight) - read(Action::RollLeft);
    c[1] += read(Action::PitchFwd) - read(Action::PitchBack);
    c[2] = read(Action::LiftUp) - read(Action::LiftDown);
    c[3] += read(Action::YawRight) - read(Action::YawLeft);
}
}  // namespace

MissionResult game_loop(Window& window,
                        Renderer& renderer,
                        Settings& settings,
                        const Mission& mission) {
    GameState state{};
    auto init_state = [&]() {
        state = {};
        state.mission = mission;
        state.pregame = mission.test_time_limit == 0;  // skip the briefing in timed tests
        state.lives = mission.starting_lives;
        state.terrain.type = mission.terrain_type;
        state.terrain.seed = static_cast<int64>(hash(state.mission.seed, uint64_t{0xf738a04c}));
    };
    GameManager game_manager;
    AI ai;
    Physics physics;
    AudioEngine audio_engine(settings);
    window.set_audio_callback([&audio_engine](int16_t* stream, int num_frames) {
        audio_engine.fill_buffer(stream, num_frames);
    });
    init_state();
    game_manager.update(state);

    Stopwatch stopwatch;
    float physics_time = 0.0f;
    ShellState pause_shell{};
    bool abandoned = false;
    while (window.frame() && !abandoned) {
        const float dt = stopwatch.lap();
        if (state.pregame) {
            renderer.draw(window, state, dt);  // mission briefing over the frozen scene
            audio_engine.update(state);
            if (window.pressed(Key::enter) || window.pressed(Key::space) || window.mouse_pressed(1))
                state.pregame = false;
            continue;
        }
        if (state.paused) {
            renderer.draw(window, state, dt);  // frozen scene
            renderer.menu_update_draw(window, pause_shell, settings, /*overlay*/ true);
            audio_engine.update(state);
            switch (pause_shell.navigation) {
                case ShellState::Navigation::None:
                    break;
                case ShellState::Navigation::Resume:
                    state.paused = false;
                    pause_shell.navigation = ShellState::Navigation::None;
                    break;
                case ShellState::Navigation::Restart:
                    init_state();
                    game_manager.reset();
                    game_manager.update(state);
                    physics_time = 0.0f;
                    pause_shell.navigation = ShellState::Navigation::None;
                    break;
                case ShellState::Navigation::Abandon:
                    abandoned = true;
                    break;
                default:
                    break;
            }
            continue;
        }
        if (mission.test_time_limit > 0 && state.tick * Physics::dt >= mission.test_time_limit)
            break;
        // Normal game loop execution
        if (!mission.test_player_ai) copy_control(window, settings, player_drone(state).control_m);
        physics_time = std::min(0.25f, physics_time + dt);
        while (physics_time >= Physics::dt) {
            ai.update(state);
            physics.tick(state);
            physics_time -= Physics::dt;
        }
        game_manager.update(state);
        renderer.draw(window, state, dt);
        audio_engine.update(state);
        auto finished = state.finish_tick.has_value() && *state.finish_tick <= state.tick;
        if ((window.pressed(Key::esc) || state.paused) && !finished) {  // pause menu
            state.paused = true;
            pause_shell.screen = ShellState::Screen::Pause;
            pause_shell.navigation = ShellState::Navigation::None;
        }
        if (window.pressed(Key::hash)) {  // instant restart shortcut
            game_manager.reset();
            init_state();
            game_manager.update(state);
            physics_time = 0.0f;
        }
        if (finished &&
            (window.pressed(Key::enter) || window.pressed(Key::space) || window.mouse_pressed(1))) {
            break;
        }
    }
    window.set_audio_callback({});  // the window's callback can still call into the audio_engine
    return state.result;
}
