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

// City
constexpr Vec<2> city_building_height_range = {10.0f, 100.0f};
constexpr Vec<2> city_skywalk_height_range = {10.0f, 80.0f};
constexpr float city_skywalk_prob = 0.5f;
constexpr float city_skywalk_clearance = 5.0f;
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

City::Pos City::nearest_cell(Vec<2> v) {
    return {(int64)std::floor(v[0] / cell_size), (int64)std::floor(v[1] / cell_size)};
}

Vec<2> City::cell_center(Pos p) {
    return {(p[0] + 0.5f) * cell_size, (p[1] + 0.5f) * cell_size};
}

City::Pos City::block_index(Pos p) {
    return {floordiv(p[0], int64(block_size)), floordiv(p[1], int64(block_size))};
}

City::Pos City::block_offset(Pos p) {
    return {floormod(p[0], int64(block_size)), floormod(p[1], int64(block_size))};
}

City::Pos City::nearest_intersection_block(Vec<2> v) {
    constexpr float block_w = float(block_size) * cell_size;
    constexpr float hw = 0.5f * float(street_size) * cell_size;
    return {std::llround((v[0] - hw) / block_w), std::llround((v[1] - hw) / block_w)};
}

Vec<2> City::intersection_center(Pos block) {
    constexpr float block_w = float(block_size) * cell_size;
    constexpr float hw = 0.5f * float(street_size) * cell_size;
    return {float(block[0]) * block_w + hw, float(block[1]) * block_w + hw};
}

Cell City::operator()(Pos p) const {
    uint64 h = hash(seed, p[0], p[1]);
    auto [bx, by] = block_index(p);
    auto [ox, oy] = block_offset(p);

    constexpr uint64 tag_x = 0x534b595f58ull;
    constexpr uint64 tag_y = 0x534b595f59ull;

    auto sample_skywalk = [&](uint64 tag, int64 a, int64 b) {
        uint64 h2 = hash(seed, tag, a, b);
        float prob = (h2 & 0xffff) * (1.0f / float(1 << 16));
        float u = ((h2 >> 16) & 0xffff) * (1.0f / float(1 << 16));
        if (prob >= city_skywalk_prob) return 0.0f;
        return city_skywalk_height_range[0] +
               u * (city_skywalk_height_range[1] - city_skywalk_height_range[0]);
    };

    float height = 0.0f;
    float skywalk_height = 0.0f;

    if (City::street_size <= ox && City::street_size <= oy) {
        float u = (h & 0xffff) * (1.0f / float(1 << 16));
        height = city_building_height_range[0] +
                 u * (city_building_height_range[1] - city_building_height_range[0]);

        auto apply_skywalk = [&](float sw) {
            if (sw > 0.0f) {
                height = std::max(height, sw + City::skywalk_thickness + city_skywalk_clearance);
            }
        };
        if (ox == City::street_size) apply_skywalk(sample_skywalk(tag_x, bx, p[1]));
        if (ox == int64(City::block_size - 1)) apply_skywalk(sample_skywalk(tag_x, bx + 1, p[1]));
        if (oy == City::street_size) apply_skywalk(sample_skywalk(tag_y, p[0], by));
        if (oy == int64(City::block_size - 1)) apply_skywalk(sample_skywalk(tag_y, p[0], by + 1));
    } else {
        if (ox < City::street_size && oy >= City::street_size) {
            skywalk_height = sample_skywalk(tag_x, bx, p[1]);
            h = hash(seed, tag_x, bx, p[1]);
        } else if (oy < City::street_size && ox >= City::street_size) {
            skywalk_height = sample_skywalk(tag_y, p[0], by);
            h = hash(seed, tag_y, p[0], by);
        }
    }
    return {height, skywalk_height, h};
}

namespace {
// Test if a sphere at p with radius r touches any building or skywalk
bool hits_building(const City& city, Vec<3> p, float r) {
    // Find the subgrid of cells that overlap the sphere footprint
    constexpr float cs = City::cell_size;
    const int64 cx0 = int64(std::floor((p[0] - r) / cs));
    const int64 cx1 = int64(std::floor((p[0] + r) / cs));
    const int64 cy0 = int64(std::floor((p[1] - r) / cs));
    const int64 cy1 = int64(std::floor((p[1] + r) / cs));
    for (int64 cy = cy0; cy <= cy1; ++cy) {
        for (int64 cx = cx0; cx <= cx1; ++cx) {
            Cell c = city({cx, cy});
            if (c.height > 0.0f) {
                Vec<3> center = {(cx + 0.5f) * cs, (cy + 0.5f) * cs, 0.5f * c.height};
                Vec<3> size = {cs, cs, c.height};
                if (intersect_box_sphere(center, size, p, r)) return true;
            }
            if (c.skywalk_height > 0.0f) {
                auto [ox, oy] = City::block_offset({cx, cy});
                constexpr float T = City::skywalk_thickness;
                Vec<3> center = {(cx + 0.5f) * cs, (cy + 0.5f) * cs, c.skywalk_height + 0.5f * T};
                Vec<3> size = (ox < int64(City::street_size) && oy >= int64(City::street_size))
                                  ? Vec<3>{cs, T, T}
                                  : Vec<3>{T, cs, T};
                if (intersect_box_sphere(center, size, p, r)) return true;
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

            // Collisions (building)
            if (hits_building(state.city, d.position, t.collision_radius)) {
                d.status = Drone::Status::Crashed;
                return;
            }

            // Alitutude-based lock-on death (player only)
            if (d.kind == Drone::Kind::Player) {
                if (d.position[2] > Drone::lock_on_level) {
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
    Pid lift = {.target = 1.0f, .lo = 0.0f, .hi = 1.0f, .kp = 16.0f, .ki = 0.06f, .kd = 16.67f};
    Pid yaw = {.target = 1.0f, .lo = -1.0f, .hi = 1.0f, .kp = 16.0f, .ki = 0.06f, .kd = 8.33f};
    Pid pitch = {.target = 1.0f, .lo = -1.0f, .hi = 1.0f, .kp = 16.0f, .ki = 0.0f, .kd = 4.17f};
    Pid roll = {.target = 1.0f, .lo = -1.0f, .hi = 1.0f, .kp = 16.0f, .ki = 0.0f, .kd = 4.17f};
    Pid lateral = {.target = 1.0f,
                   .lo = -ai_max_roll_angle,
                   .hi = ai_max_roll_angle,
                   .kp = 0.1f,
                   .ki = 0.0f,
                   .kd = 0.0f};
    PidState lift_state, yaw_state, pitch_state, roll_state, lateral_state;
    std::optional<City::Pos> current = {};
    std::optional<City::Pos> prev = {};
};

float choose_safe_altitude(const City& city, const Vec<3>& pos, Vec<2> target_pos) {
    constexpr int n_lookahead = 4;
    constexpr float margin = 5.0f;
    constexpr float min_alt = 10.0f;
    constexpr float max_alt = Drone::lock_on_level - margin;

    float alt = std::clamp(pos[2], min_alt, max_alt);

    // Find ranges that are disallowed by skywalks
    struct Range {
        float lo, hi;
    };
    std::vector<Range> ranges;
    Vec<2> cur2 = {pos[0], pos[1]};
    Vec<2> to_tgt = target_pos - cur2;
    float dist = length(to_tgt);
    Vec<2> dir = dist > 1e-3f ? normalise(to_tgt) : Vec<2>{0.0f, 0.0f};
    for (int i = 0; i <= n_lookahead; ++i) {
        float step = std::min(float(i) * City::cell_size, dist);
        Cell c = city(City::nearest_cell(cur2 + step * dir));
        if (c.skywalk_height > 0.0f) {
            ranges.push_back(
                {c.skywalk_height - margin, c.skywalk_height + City::skywalk_thickness + margin});
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

Vec<3> choose_next_waypoint(const GameState& s, DroneAI& a, const Vec<3>& pos) {
    // Initialise current/prev to the closest street intersection if unset.
    if (!a.current) {
        a.current = City::nearest_intersection_block({pos[0], pos[1]});
        a.prev = a.current;
    }

    if (length(City::intersection_center(*a.current) - Vec<2>{pos[0], pos[1]}) <=
        2.0f * City::cell_size) {
        City::Pos old_current = *a.current;
        City::Pos nbrs[4] = {{old_current[0] + 1, old_current[1]},
                             {old_current[0] - 1, old_current[1]},
                             {old_current[0], old_current[1] + 1},
                             {old_current[0], old_current[1] - 1}};
        std::vector<City::Pos> choices;
        for (auto& n : nbrs)
            if (!a.prev || n != *a.prev) choices.push_back(n);

        // Bias random walks toward the objective (defense); wander freely otherwise.
        std::vector<City::Pos> reducing;
        if (s.mission.type == Mission::Defense) {
            Vec<2> obj = s.mission.objectives[0].center;
            auto manhattan_dist = [&](City::Pos p) {
                Vec<2> c = City::intersection_center(p);
                return std::abs(c[0] - obj[0]) + std::abs(c[1] - obj[1]);
            };
            float cur_dist = manhattan_dist(old_current);
            for (auto& n : choices) {
                if (manhattan_dist(n) < cur_dist) reducing.push_back(n);
            }
        }
        City::Pos next;
        uint64 h = hash(s.city.seed, s.tick, a.id);
        float u = (h & 0xffff) * (1.0f / float(1 << 16));
        if (u >= ai_random_direction_frac && !reducing.empty()) {
            next = reducing[(h >> 16) % reducing.size()];
        } else {
            next = choices[(h >> 16) % choices.size()];
        }
        a.prev = old_current;
        a.current = next;
    }
    auto target = City::intersection_center(*a.current);
    float safe_alt = choose_safe_altitude(s.city, pos, target);
    return {target[0], target[1], safe_alt};
}

void steer_drone(const GameState& s, DroneAI& a, Drone& d) {
    Vec<3> wp = choose_next_waypoint(s, a, d.position);

    // Lift - hover at the waypoint altitude
    d.control_m[2] = pid(a.lift, a.lift_state, wp[2], d.position[2]);

    // Yaw - face the waypoint
    Vec<2> desired = normalise(Vec<2>{wp[0] - d.position[0], wp[1] - d.position[1]});
    Vec<3> fwd = rotate(d.orientation, {1.0f, 0.0f, 0.0f});
    Vec<2> fwd_xy = normalise(Vec<2>{fwd[0], fwd[1]});
    float yaw_err = std::atan2(fwd_xy[0] * desired[1] - fwd_xy[1] * desired[0],
                               fwd_xy[0] * desired[0] + fwd_xy[1] * desired[1]);
    d.control_m[3] = pid(a.yaw, a.yaw_state, 0.0f, yaw_err);

    // Pitch - aim for a target nose-down angle (when facing the waypoint)
    float target_pitch =
        std::clamp(1 - std::abs(yaw_err) / ai_yaw_deadband, 0.0f, 1.0f) * ai_pitch_angle;
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
                (s.mission.player_ai && d.kind == Drone::Kind::Player)) {
                steer_drone(s, a, d);
            }
        });
}
