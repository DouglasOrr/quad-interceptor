#include <algorithm>
#include <optional>

#include "game.hpp"

namespace {
////////////////////////////////////////////////////////////////////////////////
// Settings

constexpr Vec<3> up = {0.0f, 0.0f, 1.0f};
constexpr Vec<3> gravity = {0.0f, 0.0f, -10.0f};

// Drone
constexpr float explosion_radius = 5.0f;
constexpr float lock_on_rate = 1 / 15.0f;
constexpr float lock_on_reset_rate = 1 / 30.0f;

// Structural-choice hash tags (arbitrary bytes; stable ids, not text)
constexpr uint64 tag_x = 0x534b595f58ull;
constexpr uint64 tag_y = 0x534b595f59ull;
constexpr uint64 tag_stilts = 0x5354494c5453ull;
constexpr uint64 tag_shade = 0x5348414445ull;
constexpr uint64 tag_wire = 0x57495245ull;

// City: vertical gap between a skywalk slab and the building top below it.
constexpr float skywalk_clearance = 5.0f;
////////////////////////////////////////////////////////////////////////////////
// PID

struct PidState {
    float integral = 0.0f;
    float prev = 0.0f;
};

float pid(const Pid& s, PidState& st, float control, float actual) {
    float error = s.target * control - actual;
    st.integral += error * Physics::dt;
    st.integral = std::clamp(st.integral, -10.0f, 10.0f);  // anti-windup
    float delta = (error - st.prev) / Physics::dt;
    st.prev = error;
    return std::clamp(s.kp * error + s.ki * st.integral + s.kd * delta, s.lo, s.hi);
}
}  // namespace

std::array<Vec<3>, 4> DroneType::motor_offsets_from_radius(float radius) {
    const float r = radius / M_SQRT2f;
    return {{
        {{r, r, 0.0f}},    // FL
        {{r, -r, 0.0f}},   // FR
        {{-r, r, 0.0f}},   // BL
        {{-r, -r, 0.0f}},  // BR
    }};
}

////////////////////////////////////////////////////////////////////////////////
// Terrain

Terrain::Pos Terrain::nearest_cell(Vec<2> v) {
    return {(int64)std::floor(v[0] / cell_size), (int64)std::floor(v[1] / cell_size)};
}
Vec<2> Terrain::cell_center(Pos p) {
    return {(p[0] + 0.5f) * cell_size, (p[1] + 0.5f) * cell_size};
}

namespace {
// City block helpers
Terrain::Pos city_block_index(Terrain::Pos p, uint block) {
    return {floordiv(p[0], int64(block)), floordiv(p[1], int64(block))};
}
Terrain::Pos city_block_offset(Terrain::Pos p, uint block) {
    return {floormod(p[0], int64(block)), floormod(p[1], int64(block))};
}
float city_block_w(const TerrainType::City& city) {
    return float(city.block_size) * Terrain::cell_size;
}
float city_street_hw(const TerrainType::City& city) {
    return 0.5f * float(city.street_size) * Terrain::cell_size;
}
Terrain::Pos nearest_node(const TerrainType::City& city, Vec<2> v) {
    const float bw = city_block_w(city);
    const float hw = city_street_hw(city);
    return {std::llround((v[0] - hw) / bw), std::llround((v[1] - hw) / bw)};
}
Vec<2> node_center(const TerrainType::City& city, Terrain::Pos p) {
    const float bw = city_block_w(city);
    const float hw = city_street_hw(city);
    return {float(p[0]) * bw + hw, float(p[1]) * bw + hw};
}

// Plains power-line lattice.
bool plains_row_active(const TerrainType::Plains& t, int64 seed, int64 cy) {
    const int64 period = int64(t.pylon_row_period);
    if (floormod(cy, period) != 0) return false;
    return hash_u(hash(seed, tag_wire, floordiv(cy, period)), 0) < t.pylon_run_prob;
}
bool plains_col_active(const TerrainType::Plains& t, int64 seed, int64 cx) {
    const int64 period = int64(t.pylon_row_period);
    if (floormod(cx, period) != 0) return false;
    return hash_u(hash(seed, tag_wire, floordiv(cx, period)), 16) < t.pylon_run_prob;
}

Terrain::Cell sample_plains(const Terrain& terr, Terrain::Pos p) {
    const auto& t = terr.type.plains;
    const int64 stride = int64(t.pylon_stride);
    uint64 h = hash(terr.seed, p[0], p[1]);

    bool on_row_line = plains_row_active(t, terr.seed, p[1]);
    bool on_col_line = plains_col_active(t, terr.seed, p[0]);
    bool xrun = on_row_line && floormod(p[0], stride) == 0;
    bool yrun = on_col_line && floormod(p[1], stride) == 0;

    Terrain::Cell cell{};  // value-init: type = Empty
    cell.hash = h;
    if (on_row_line || on_col_line) {
        // Pylons on run grid positions; the rest of a run is a cleared corridor.
        if (xrun || yrun) {
            cell.type = Terrain::CellType::Pole;
            cell.pole.radius = t.pylon_radius;
            cell.pole.top = t.pylon_height;
            cell.pole.pylon_x = xrun;
            cell.pole.pylon_y = yrun;
        }
    } else if (hash_u(h, 0) < t.pole_prob) {
        cell.type = Terrain::CellType::Pole;
        cell.pole.radius = hash_u(h, 16, t.pole_radius_range[0], t.pole_radius_range[1]);
        cell.pole.top = hash_u(h, 32, t.pole_height_range[0], t.pole_height_range[1]);
        cell.pole.pylon_x = false;
        cell.pole.pylon_y = false;
    }
    return cell;
}
Terrain::Cell sample_city(const Terrain& terr, Terrain::Pos p) {
    uint64 h = hash(terr.seed, p[0], p[1]);
    const auto& ct = terr.type.city;

    auto [bx, by] = city_block_index(p, ct.block_size);
    auto [ox, oy] = city_block_offset(p, ct.block_size);

    // A stilted block: solid lower floors [0, podium], a fly-through gap
    // [podium, podium+gap], then variable per-cell buildings on top.
    // Returns {podium, gap}; empty (podium==0) = block on the ground.
    auto sample_block_base = [&](Terrain::Pos block) -> std::tuple<float, float> {
        uint64 h2 = hash(terr.seed, tag_stilts, block[0], block[1]);
        if (hash_u(h2, 0) >= ct.stilt_prob) return {0.0f, 0.0f};
        float podium = hash_u(h2, 16, ct.stilt_podium_range[0], ct.stilt_podium_range[1]);
        float gap = hash_u(h2, 32, ct.stilt_gap_range[0], ct.stilt_gap_range[1]);
        return {podium, gap};
    };

    // This block's podium and the top of its gap (= building box bottom).
    auto [this_podium, this_gap] = sample_block_base({bx, by});
    float this_base = this_podium + this_gap;

    // Reject a skywalk that would hang inside the gap of a neighbouring stilted
    // block: check the block on the far side of the walkway and this block.
    auto sample_skywalk = [&](uint64 tag, int64 a, int64 b, Terrain::Pos blk_a) {
        uint64 h2 = hash(terr.seed, tag, a, b);
        if (hash_u(h2, 0) >= ct.skywalk_prob) return 0.0f;
        float sw = hash_u(h2, 16, ct.skywalk_height_range[0], ct.skywalk_height_range[1]);
        auto [pod, gap] = sample_block_base(blk_a);
        if (sw < pod + gap || sw < this_base) return 0.0f;
        return sw;
    };

    Terrain::Cell cell{};                                // value-init: type = Empty
    if (ct.street_size <= ox && ct.street_size <= oy) {  // Building
        float building = hash_u(h, 0, ct.building_height_range[0], ct.building_height_range[1]);
        cell.type = Terrain::CellType::Building;
        cell.hash = hash(terr.seed, tag_shade, p[0], p[1]);
        cell.building.base = this_base;
        cell.building.podium = this_podium;
        cell.building.top = this_base + building;

        auto apply_skywalk = [&](float sw) {
            if (sw > 0.0f) {
                cell.building.top =
                    std::max(cell.building.top, sw + ct.skywalk_thickness + skywalk_clearance);
            }
        };
        if (ox == ct.street_size) apply_skywalk(sample_skywalk(tag_x, bx, p[1], {bx - 1, by}));
        if (ox == int64(ct.block_size - 1))
            apply_skywalk(sample_skywalk(tag_x, bx + 1, p[1], {bx + 1, by}));
        if (oy == ct.street_size) apply_skywalk(sample_skywalk(tag_y, p[0], by, {bx, by - 1}));
        if (oy == int64(ct.block_size - 1))
            apply_skywalk(sample_skywalk(tag_y, p[0], by + 1, {bx, by + 1}));

    } else {  // Street
        if (ox < ct.street_size && oy >= ct.street_size) {
            float sw = sample_skywalk(tag_x, bx, p[1], {bx - 1, by});
            if (sw > 0.0f) {
                cell.type = Terrain::CellType::Skywalk;
                cell.hash = hash(terr.seed, tag_x, bx);  // constant across the strip
                cell.skywalk.bottom = sw;
                cell.skywalk.axis_x = true;  // crosses the y-street -> runs along x
            }
        } else if (oy < ct.street_size && ox >= ct.street_size) {
            float sw = sample_skywalk(tag_y, p[0], by, {bx, by - 1});
            if (sw > 0.0f) {
                cell.type = Terrain::CellType::Skywalk;
                cell.hash = hash(terr.seed, tag_y, by);  // constant across the strip
                cell.skywalk.bottom = sw;
                cell.skywalk.axis_x = false;  // runs along y
            }
        }
    }
    return cell;
}
}  // namespace

Vec<2> TerrainType::City::snap_intersection(Vec<2> v) const {
    return node_center(*this, nearest_node(*this, v));
}

Terrain::Cell Terrain::operator()(Pos p) const {
    switch (type.kind) {
        case TerrainType::Kind::Plains:
            return sample_plains(*this, p);
        case TerrainType::Kind::City:
            return sample_city(*this, p);
        default:
            gassert(false && "unexpected TerrainType");
            return {.type = Terrain::CellType::Empty, .hash = 0, .empty = {}};
    }
}

namespace {
// Test if a sphere at p with radius r touches any terrain geometry
bool hits_terrain(const Terrain& terr, Vec<3> p, float r) {
    // Find the subgrid of cells that overlap the sphere footprint
    constexpr float cs = Terrain::cell_size;
    const int64 cx0 = int64(std::floor((p[0] - r) / cs));
    const int64 cx1 = int64(std::floor((p[0] + r) / cs));
    const int64 cy0 = int64(std::floor((p[1] - r) / cs));
    const int64 cy1 = int64(std::floor((p[1] + r) / cs));
    for (int64 cy = cy0; cy <= cy1; ++cy) {
        for (int64 cx = cx0; cx <= cx1; ++cx) {
            auto c = terr({cx, cy});
            Vec<2> pc = Terrain::cell_center({cx, cy});
            switch (c.type) {
                case Terrain::CellType::Building: {
                    const auto& b = c.building;
                    // Top
                    if (intersect_box_sphere({pc[0], pc[1], 0.5f * (b.base + b.top)},
                                             {cs, cs, b.top - b.base}, p, r))
                        return true;
                    // Podium
                    if (b.podium > 0.0f) {
                        if (intersect_box_sphere({pc[0], pc[1], 0.5f * b.podium},
                                                 {cs, cs, b.podium}, p, r))
                            return true;
                        // Stilts
                        if (b.base > b.podium) {
                            if (intersect_capsule_sphere({pc[0], pc[1], b.podium},
                                                         {pc[0], pc[1], b.base},
                                                         TerrainType::City::pillar_radius, p, r))
                                return true;
                        }
                    }
                    break;
                }
                case Terrain::CellType::Skywalk: {
                    const float T = TerrainType::City::skywalk_thickness;
                    Vec<3> center = {pc[0], pc[1], c.skywalk.bottom + 0.5f * T};
                    Vec<3> size = c.skywalk.axis_x ? Vec<3>{cs, T, T} : Vec<3>{T, cs, T};
                    if (intersect_box_sphere(center, size, p, r)) return true;
                    break;
                }
                case Terrain::CellType::Pole: {
                    const auto& pl = c.pole;
                    if (intersect_capsule_sphere({pc[0], pc[1], 0.0f}, {pc[0], pc[1], pl.top},
                                                 pl.radius, p, r))
                        return true;
                    break;
                }
                case Terrain::CellType::Empty:
                    break;
            }
        }
    }
    return false;
}
float attitude_roll(const Quat& orientation) {  // + = rolled right (as control_m)
    Vec<3> left = rotate(orientation, {0.0f, 1.0f, 0.0f});
    return std::asin(std::clamp(left[2], -1.0f, 1.0f));
}
float attitude_pitch(const Quat& orientation) {  // + = nose down (as control_m)
    Vec<3> fwd = rotate(orientation, {1.0f, 0.0f, 0.0f});
    return -std::asin(std::clamp(fwd[2], -1.0f, 1.0f));
}
}  // namespace

////////////////////////////////////////////////////////////////////////////////
// Physics

struct Physics::State {
    struct Drone {
        uint id;
        PidState roll_rate, pitch_rate, lift_rate, yaw_rate;
        PidState roll_angle, pitch_angle;
    };
    std::vector<Drone> drones;
};

Physics::Physics() : state(std::make_unique<State>()) {}
Physics::~Physics() = default;

void Physics::tick(GameState& state) {
    ++state.tick;
    auto& player = player_drone(state);
    sync_parallel(
        state.drones, this->state->drones,
        [](const Drone& d) {
            State::Drone ps;
            ps.id = d.id;
            return ps;
        },
        [&](Drone& d, State::Drone& ps) {
            const auto& t = d.type;
            auto R = mat_from(d.orientation);
            auto Rt = transpose(R);
            auto ang_vel_m = mul(t.inv_inertia_m, mul(Rt, d.ang_momentum));  // actual rates (model)

            // Collisions (terrain).
            if ((d.kind != Drone::Kind::Enemy || state.mission.test_enemy_crashes) &&
                hits_terrain(state.terrain, d.position, t.collision_radius)) {
                d.status = Drone::Status::Crashed;
                return;
            }

            // Alitutude-based lock-on death (player only)
            if (d.kind == Drone::Kind::Player &&
                state.terrain.type.kind == TerrainType::Kind::City) {
                if (d.position[2] > state.terrain.type.city.lock_on_level) {
                    d.lock_on += lock_on_rate * dt;
                    if (d.lock_on >= 1.0f) {
                        d.status = Drone::Status::Crashed;
                        return;
                    }
                } else {
                    d.lock_on = std::max(0.0f, d.lock_on - lock_on_reset_rate * dt);
                }
            }

            // Collisions (drones: enemy against player)
            if (d.kind == Drone::Kind::Enemy && player.status == Drone::Status::Alive) {
                if (length(d.position - player.position) < explosion_radius) {
                    d.status = Drone::Status::Detonated;
                    player.status = Drone::Status::Detonated;
                }
            }

            // Collisions (floor)
            if (d.position[2] - t.height <= 0.0f) {
                d.position[2] = t.height;
                constexpr auto floor_drag = 8.0f;
                d.velocity[0] -= (dt * floor_drag) * sign(d.velocity[0]);
                d.velocity[1] -= (dt * floor_drag) * sign(d.velocity[1]);
                if (d.velocity[2] < 0.0f) {
                    d.velocity[2] = 0.0f;
                    Vec<3> current_up = mul(R, {0.0f, 0.0f, 1.0f});
                    Vec<3> target_up = {0.0f, 0.0f, 1.0f};
                    d.ang_momentum +=
                        (dt * 50.0f) * cross(current_up, target_up) - (dt * 10.0f) * d.ang_momentum;
                }
            }

            // Controls
            auto [roll, pitch, lift, yaw] = d.control_m.data;
            if (t.mix_roll_to_yaw) yaw += t.mix_roll_to_yaw * roll;

            // Controls - angle PIDs
            if (t.pid_roll_angle)
                roll = pid(*t.pid_roll_angle, ps.roll_angle, roll, attitude_roll(d.orientation));
            if (t.pid_pitch_angle)
                pitch =
                    pid(*t.pid_pitch_angle, ps.pitch_angle, pitch, attitude_pitch(d.orientation));
            // Controls - rate PIDs
            if (t.pid_roll_rate) roll = pid(*t.pid_roll_rate, ps.roll_rate, roll, ang_vel_m[0]);
            if (t.pid_pitch_rate)
                pitch = pid(*t.pid_pitch_rate, ps.pitch_rate, pitch, ang_vel_m[1]);
            if (t.pid_lift_rate) lift = pid(*t.pid_lift_rate, ps.lift_rate, lift, d.velocity[2]);
            if (t.pid_yaw_rate) yaw = pid(*t.pid_yaw_rate, ps.yaw_rate, yaw, -ang_vel_m[2]);
            // Motor mixer
            auto cFL = lift + yaw - pitch + roll;
            auto cFR = lift - yaw - pitch - roll;
            auto cBL = lift - yaw + pitch + roll;
            auto cBR = lift + yaw + pitch - roll;

            auto motor_speed_scale = 3.0f;
            auto _speed = [&](float c) { return std::min(1.0f, std::abs(c) / motor_speed_scale); };
            d.motor_speed = {_speed(cFL), _speed(cFR), _speed(cBL), _speed(cBR)};

            // Forces
            auto force = t.mass * gravity;
            force += t.motor_force * (cFL + cFR + cBL + cBR) * mul(R, up);
            force -= mul(R, mul(t.damping_m, mul(Rt, d.velocity)));

            // Torques
            auto torque_m = t.motor_torque * (-cFL + cFR + cBL - cBR) * up;
            torque_m += cross(t.motor_offsets_m[0], cFL * t.motor_force * up);
            torque_m += cross(t.motor_offsets_m[1], cFR * t.motor_force * up);
            torque_m += cross(t.motor_offsets_m[2], cBL * t.motor_force * up);
            torque_m += cross(t.motor_offsets_m[3], cBR * t.motor_force * up);
            auto torque = mul(R, torque_m);

            // Dynamics
            d.velocity += (dt / t.mass) * force;
            d.position += dt * d.velocity;
            auto ang_vel = mul(R, ang_vel_m);
            d.ang_momentum += dt * (torque - mul(R, mul(t.ang_damping_m, ang_vel_m)));
            d.orientation =
                normalise(d.orientation + 0.5 * dt * (quat_pure(ang_vel) * d.orientation));
        });
}

////////////////////////////////////////////////////////////////////////////////
// AI

namespace {
constexpr float ai_yaw_deadband = 45.0f * deg_to_rad;
constexpr float ai_pitch_angle = 30.0f * deg_to_rad;
constexpr float ai_max_roll_angle = 60.0f * deg_to_rad;
constexpr float ai_random_direction_frac = 0.5f;

struct DroneAI {
    uint id;
    Pid lift = {.target = 1.0f, .lo = 0.0f, .hi = 1.0f, .kp = 16.0f, .ki = 0.06f, .kd = 16.0f};
    Pid yaw = {.target = 1.0f, .lo = -1.0f, .hi = 1.0f, .kp = 8.0f, .ki = 0.06f, .kd = 4.0f};
    Pid pitch = {.target = 1.0f, .lo = -1.0f, .hi = 1.0f, .kp = 16.0f, .ki = 0.0f, .kd = 2.0f};
    Pid roll = {.target = 1.0f, .lo = -1.0f, .hi = 1.0f, .kp = 25.0f, .ki = 0.0f, .kd = 8.0f};
    Pid lateral = {.target = 1.0f,
                   .lo = -ai_max_roll_angle,
                   .hi = ai_max_roll_angle,
                   .kp = 0.1f,
                   .ki = 0.0f,
                   .kd = 0.0f};
    PidState lift_state, yaw_state, pitch_state, roll_state, lateral_state;
    // City: random walk on the intersection lattice.
    std::optional<Terrain::Pos> current = {};
    std::optional<Terrain::Pos> prev = {};
    // Plains: free-flight goal, re-picked on arrival.
    std::optional<Vec<2>> goal = {};
};

float choose_safe_altitude(const Terrain& terr, const Vec<3>& pos, Vec<2> target_pos) {
    constexpr int n_lookahead = 4;
    constexpr float margin = 5.0f;
    constexpr float min_alt = 10.0f;
    const float max_alt =
        terr.type.kind == TerrainType::Kind::City ? terr.type.city.lock_on_level - margin : 60.0f;

    float alt = std::clamp(pos[2], min_alt, max_alt);

    // Find ranges that are disallowed by terrain along the way: skywalks, poles
    struct Range {
        float lo, hi;
    };
    std::vector<Range> ranges;
    Vec<2> cur2 = {pos[0], pos[1]};
    Vec<2> to_tgt = target_pos - cur2;
    float dist = length(to_tgt);
    Vec<2> dir = dist > 1e-3f ? normalise(to_tgt) : Vec<2>{0.0f, 0.0f};
    for (int i = 0; i <= n_lookahead; ++i) {
        float step = std::min(float(i) * Terrain::cell_size, dist);
        auto c = terr(Terrain::nearest_cell(cur2 + step * dir));
        if (c.type == Terrain::CellType::Skywalk) {
            ranges.push_back({c.skywalk.bottom - margin,
                              c.skywalk.bottom + TerrainType::City::skywalk_thickness + margin});
        } else if (c.type == Terrain::CellType::Pole) {
            ranges.push_back({0.0f, c.pole.top + margin});
        }
    }
    std::ranges::sort(ranges, [](const auto& a, const auto& b) { return a.lo < b.lo; });

    // Merge overlaps
    size_t n = 0;
    for (const auto& r : ranges) {
        if (n > 0 && r.lo <= ranges[n - 1].hi) {
            ranges[n - 1].hi = std::max(ranges[n - 1].hi, r.hi);
        } else {
            ranges[n++] = r;
        }
    }
    ranges.resize(n);

    // If `alt` lies in a disallowed range, go up or down to the nearer boundary
    for (const auto& r : ranges) {
        if (alt >= r.lo && alt <= r.hi) {
            bool prefer_lo = alt - r.lo < r.hi - alt || r.hi > max_alt;
            return (prefer_lo && r.lo >= min_alt) ? r.lo : std::min(r.hi, max_alt);
        }
    }
    return alt;
}

Vec<3> choose_city_waypoint(const GameState& s, DroneAI& a, const Vec<3>& pos) {
    // Initialise current/prev to the closest intersection node if unset.
    if (!a.current) {
        a.current = nearest_node(s.terrain.type.city, {pos[0], pos[1]});
        a.prev = a.current;
    }

    if (length(node_center(s.terrain.type.city, *a.current) - Vec<2>{pos[0], pos[1]}) <=
        Terrain::cell_size) {
        Terrain::Pos old_current = *a.current;
        Terrain::Pos nbrs[4] = {{old_current[0] + 1, old_current[1]},
                                {old_current[0] - 1, old_current[1]},
                                {old_current[0], old_current[1] + 1},
                                {old_current[0], old_current[1] - 1}};
        std::vector<Terrain::Pos> choices;
        for (auto& n : nbrs)
            if (!a.prev || n != *a.prev) choices.push_back(n);

        // Bias random walks toward the objective (defense); wander freely otherwise.
        std::vector<Terrain::Pos> reducing;
        if (s.mission.type == Mission::Defense) {
            Vec<2> obj = s.mission.objectives[0].center;
            auto manhattan_dist = [&](Terrain::Pos p) {
                Vec<2> c = node_center(s.terrain.type.city, p);
                return std::abs(c[0] - obj[0]) + std::abs(c[1] - obj[1]);
            };
            float cur_dist = manhattan_dist(old_current);
            for (auto& n : choices) {
                if (manhattan_dist(n) < cur_dist) reducing.push_back(n);
            }
        }
        Terrain::Pos next;
        uint64 h = hash(s.terrain.seed, s.tick, a.id);
        if (hash_u(h, 0) >= ai_random_direction_frac && !reducing.empty()) {
            next = reducing[(h >> 16) % reducing.size()];
        } else {
            next = choices[(h >> 16) % choices.size()];
        }
        a.prev = old_current;
        a.current = next;
    }
    auto target = node_center(s.terrain.type.city, *a.current);
    float safe_alt = choose_safe_altitude(s.terrain, pos, target);
    return {target[0], target[1], safe_alt};
}

Vec<3> choose_plains_waypoint(const GameState& s, DroneAI& a, const Vec<3>& pos) {
    // Pick a direction and fly that leg, re-picking on arrival.
    constexpr Vec<2> leg_range = {40.0f, 60.0f};  // leg length, in cells
    Vec<2> p2 = {pos[0], pos[1]};
    if (!a.goal || length(*a.goal - p2) <= Terrain::cell_size) {
        uint64 h = hash(s.terrain.seed, s.tick, a.id);
        float angle = 0.0f;
        if (s.mission.type == Mission::Defense && !s.mission.objectives.empty()) {
            Vec<2> to_obj = s.mission.objectives[0].center - p2;
            angle = std::atan2(to_obj[1], to_obj[0]) + hash_u(h, 0, -0.25f, 0.25f);
        } else {
            angle = hash_u(h, 0, 0.0f, 2.0f * pi);
        }
        float leg = hash_u(h, 16, leg_range[0], leg_range[1]) * Terrain::cell_size;
        a.goal = p2 + leg * Vec<2>{std::cos(angle), std::sin(angle)};
    }
    float safe_alt = choose_safe_altitude(s.terrain, pos, *a.goal);
    return {(*a.goal)[0], (*a.goal)[1], safe_alt};
}

void steer_drone(const GameState& s, DroneAI& a, Drone& d) {
    Vec<3> wp = (s.terrain.type.kind == TerrainType::Kind::Plains)
                    ? choose_plains_waypoint(s, a, d.position)
                    : choose_city_waypoint(s, a, d.position);

    // Lift - hover at the waypoint altitude
    d.control_m[2] = pid(a.lift, a.lift_state, wp[2], d.position[2]);

    // Yaw - face the waypoint
    Vec<2> desired = normalise(Vec<2>{wp[0] - d.position[0], wp[1] - d.position[1]});
    Vec<3> fwd = rotate(d.orientation, {1.0f, 0.0f, 0.0f});
    Vec<2> fwd_xy = normalise(Vec<2>{fwd[0], fwd[1]});
    float yaw_err = std::atan2(fwd_xy[0] * desired[1] - fwd_xy[1] * desired[0],
                               fwd_xy[0] * desired[0] + fwd_xy[1] * desired[1]);
    d.control_m[3] = pid(a.yaw, a.yaw_state, 0.0f, yaw_err);

    // Pitch - aim for a target nose-down angle (when facing the waypoint).
    float target_pitch = std::clamp(1 - std::abs(yaw_err) / ai_yaw_deadband, 0.0f, 1.0f) *
                         ai_pitch_angle * s.mission.enemy_tilt;
    auto pitch = attitude_pitch(d.orientation);
    d.control_m[1] = pid(a.pitch, a.pitch_state, target_pitch, pitch);

    // Lateral Vel > Roll - aim for zero lateral motion.
    float lat_vel = -d.velocity[0] * fwd_xy[1] + d.velocity[1] * fwd_xy[0];
    float target_roll = -pid(a.lateral, a.lateral_state, 0.0f, lat_vel);
    auto roll = attitude_roll(d.orientation);
    d.control_m[0] = pid(a.roll, a.roll_state, target_roll, roll);
}
}  // namespace

struct AI::State {
    std::vector<DroneAI> drones;
};

AI::AI() : state(std::make_unique<State>()) {}
AI::~AI() = default;

void AI::update(GameState& s) {
    sync_parallel(
        s.drones, state->drones,
        [](const Drone& d) {
            DroneAI a;
            a.id = d.id;
            return a;
        },
        [&](Drone& d, DroneAI& a) {
            if (d.status != Drone::Status::Alive) return;
            if (d.kind == Drone::Kind::Enemy ||
                (s.mission.test_player_ai && d.kind == Drone::Kind::Player)) {
                steer_drone(s, a, d);
            }
        });
}
