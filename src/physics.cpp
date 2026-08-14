#include <algorithm>
#include <optional>

#include "game.hpp"

namespace {
////////////////////////////////////////////////////////////////////////////////
// Settings

constexpr Vec<3> up = {0.0f, 0.0f, 1.0f};
constexpr Vec<3> gravity = {0.0f, 0.0f, -10.0f};

// Drone
constexpr float body_mass = 1.0f;
constexpr Mat3 damping_m = mat_diag(0.2f, 0.2f, 1.0f);
constexpr Mat3 body_inv_inertia_m = mat_diag(0.4f, 0.4f, 1.5f);
constexpr float motor_force = 8.0f;
constexpr float motor_torque = 0.6f;
constexpr Mat3 ang_damping_m = mat_diag(0.4f, 0.4f, 0.4f);

constexpr float explosion_radius = 5.0f;
constexpr float lock_on_rate = 1 / 15.0f;
constexpr float lock_on_reset_rate = 1 / 30.0f;

// City
constexpr Vec<2> city_building_height_range = {10.0f, 100.0f};

////////////////////////////////////////////////////////////////////////////////
// Utilities

int64 floordiv(int64 a, int64 b) {
    return a >= 0 ? a / b : (a - (b - 1)) / b;
}

uint64 hash_mix(uint64 x) {
    x ^= x >> 32;
    x *= 0xd6e8feb86659fd93ULL;
    x ^= x >> 32;
    x *= 0xd6e8feb86659fd93ULL;
    x ^= x >> 32;
    return x;
}

template <typename... Args>
uint64_t hash(uint64_t seed, Args... args) {
    uint64_t h = seed;
    ((h = hash_mix(h ^ uint64_t(args))), ...);
    return h;
}
}  // namespace

float Pid::operator()(float control, float actual, float dt) {
    if (!enabled) return std::clamp(control, lo, hi);
    float error = target * control - actual;
    integral += error * dt;
    integral = std::clamp(integral, -10.0f, 10.0f);  // anti-windup
    float delta = (error - prev) / dt;
    prev = error;
    return std::clamp(kp * error + ki * integral + kd * delta, lo, hi);
}

City::Pos City::nearest_cell(Vec<2> v) {
    return {(int64)std::floor(v[0] / cell_size), (int64)std::floor(v[1] / cell_size)};
}

Vec<2> City::cell_center(Pos p) {
    return {(p[0] + 0.5f) * cell_size, (p[1] + 0.5f) * cell_size};
}

Cell City::operator()(Pos p) const {
    uint64 h = hash(seed, p[0], p[1]);
    int64 ox = p[0] - floordiv(p[0], int64(City::block_size)) * int64(City::block_size);
    int64 oy = p[1] - floordiv(p[1], int64(City::block_size)) * int64(City::block_size);

    float height = 0.0f;  // street
    if (City::street_size <= ox && City::street_size <= oy) {
        float u = (h & 0xffffff) * (1.0f / float(1 << 24));
        height = city_building_height_range[0] +
                 u * (city_building_height_range[1] - city_building_height_range[0]);
    }
    return {height, h};
}

namespace {
// Test if a sphere at p with radius r touches any building
bool hits_building(const City& city, Vec<3> p, float r) {
    // Find the subgrid of cells that overlap the sphere footprint
    constexpr float cs = float(City::cell_size);
    const int64 cx0 = floordiv(int64(std::floor(p[0] - r)), int64(cs));
    const int64 cx1 = floordiv(int64(std::floor(p[0] + r)), int64(cs));
    const int64 cy0 = floordiv(int64(std::floor(p[1] - r)), int64(cs));
    const int64 cy1 = floordiv(int64(std::floor(p[1] + r)), int64(cs));
    for (int64 cy = cy0; cy <= cy1; ++cy) {
        for (int64 cx = cx0; cx <= cx1; ++cx) {
            float h = city({cx, cy}).height;
            if (h <= 0.0f) continue;
            // Closest point on the building AABB to the sphere centre
            Vec<3> q = {
                std::clamp(p[0], float(cx) * cs, float(cx + 1) * cs),
                std::clamp(p[1], float(cy) * cs, float(cy + 1) * cs),
                std::clamp(p[2], 0.0f, h),
            };
            Vec<3> d = q - p;
            if (d[0] * d[0] + d[1] * d[1] + d[2] * d[2] < r * r) return true;
        }
    }
    return false;
}
}  // namespace

Drone Drone::create(Vec<3> position) {
    Drone d;
    d.position = position;
    d.velocity = {};
    d.orientation = {0.0f, 0.0f, 0.0f, 1.0f};
    d.ang_momentum = {};
    d.status = Drone::Status::Alive;
    d.lock_on = 0;
    d.control_m = {};
    d.motor_speed = {};
    Pid rot = {.target = 2.5f, .lo = -1.0f, .hi = 1.0f, .kp = 5.0f, .ki = 3.0f, .kd = 0.0f};
    Pid lift = {.target = 10.0f, .lo = 0.0f, .hi = 1.0f, .kp = 5.0f, .ki = 3.0f, .kd = 0.0f};
    lift.enabled = false;
    d.pid = {rot, rot, lift, rot};  // roll, pitch, lift, yaw
    return d;
}

////////////////////////////////////////////////////////////////////////////////
// Physics

void Physics::tick(GameState& state) {
    ++state.tick;
    foreach_drone(state, [&](size_t d_index, Drone& d) {
        auto R = mat_from(d.orientation);
        auto Rt = transpose(R);
        auto ang_vel_m = mul(body_inv_inertia_m, mul(Rt, d.ang_momentum));  // actual rates (model)

        // Collisions (building)
        if (hits_building(state.city, d.position, Drone::collision_radius)) {
            d.status = Drone::Status::Crashed;
            return;
        }

        // Alitutude-based lock-on death (player only)
        if (d_index == 0) {
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

        // Collisions (drones)
        if (d_index != 0 && state.drones[0].status == Drone::Status::Alive) {
            if (length(d.position - state.drones[0].position) < explosion_radius) {
                d.status = Drone::Status::Detonated;
                state.drones[0].status = Drone::Status::Detonated;
            }
        }

        // Collisions (floor)
        if (d.position[2] - Drone::height <= 0.0f) {
            d.position[2] = Drone::height;
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

        // Controls - PID
        auto lift = d.pid[2](d.control_m[2], d.velocity[2], dt);
        auto cw = d.pid[3](d.control_m[3], -ang_vel_m[2], dt);
        auto roll = d.pid[0](d.control_m[0], ang_vel_m[0], dt);
        auto pitch = d.pid[1](d.control_m[1], ang_vel_m[1], dt);

        // Motor mixer
        auto cFL = lift + cw - pitch + roll;
        auto cFR = lift - cw - pitch - roll;
        auto cBL = lift - cw + pitch + roll;
        auto cBR = lift + cw + pitch - roll;

        auto _speed = [](float c) { return std::min(1.0f, std::abs(c) / 3.0f); };
        d.motor_speed = {_speed(cFL), _speed(cFR), _speed(cBL), _speed(cBR)};

        // Forces
        auto force = body_mass * gravity;
        force += motor_force * (cFL + cFR + cBL + cBR) * mul(R, up);
        force -= mul(R, mul(damping_m, mul(Rt, d.velocity)));

        // Torques
        auto torque_m = motor_torque * (-cFL + cFR + cBL - cBR) * up;
        torque_m += cross(Drone::motor_offsets_m[0], cFL * motor_force * up);
        torque_m += cross(Drone::motor_offsets_m[1], cFR * motor_force * up);
        torque_m += cross(Drone::motor_offsets_m[2], cBL * motor_force * up);
        torque_m += cross(Drone::motor_offsets_m[3], cBR * motor_force * up);
        auto torque = mul(R, torque_m);

        // Dynamics
        d.velocity += (dt / body_mass) * force;
        d.position += dt * d.velocity;
        auto ang_vel = mul(R, ang_vel_m);
        d.ang_momentum += dt * (torque - mul(R, mul(ang_damping_m, ang_vel_m)));
        d.orientation = normalise(d.orientation + 0.5 * dt * (quat_pure(ang_vel) * d.orientation));
    });
}

////////////////////////////////////////////////////////////////////////////////
// AI

namespace {
constexpr float ai_hover_altitude = 20.0f;
constexpr float ai_yaw_deadband = 45.0f * deg_to_rad;
constexpr float ai_pitch_angle = 30.0f * deg_to_rad;
constexpr float ai_max_roll_angle = 60.0f * deg_to_rad;

struct DroneAI {
    Pid lift = {.target = 1.0f, .lo = 0.0f, .hi = 1.0f, .kp = 16.0f, .ki = 1.0f, .kd = 1.0f};
    Pid yaw = {.target = 1.0f, .lo = -1.0f, .hi = 1.0f, .kp = 16.0f, .ki = 1.0f, .kd = 0.5f};
    Pid pitch = {.target = 1.0f, .lo = -1.0f, .hi = 1.0f, .kp = 16.0f, .ki = 0.0f, .kd = 0.25f};
    Pid roll = {.target = 1.0f, .lo = -1.0f, .hi = 1.0f, .kp = 16.0f, .ki = 0.0f, .kd = 0.25f};
    Pid lateral = {.target = 1.0f,
                   .lo = -ai_max_roll_angle,
                   .hi = ai_max_roll_angle,
                   .kp = 0.1f,
                   .ki = 0.0f,
                   .kd = 0.0f};
    std::optional<City::Pos> current;
    std::optional<City::Pos> prev;
};

// Street corners live at cell coordinates ≡ 1 (mod City::block_size), i.e. the
// centre of a street_size-wide street. Advances to a new corner (excluding the
// previous, to avoid doubling back) when within 2*cell_size, and returns the
// world target to fly to.
Vec<3> choose_next_waypoint(const GameState& s, size_t i, DroneAI& a, const Vec<3>& pos) {
    // Initialise current/prev to the closest street corner if unset.
    if (!a.current) {
        auto c = City::nearest_cell({pos[0], pos[1]});
        int64 bx = (int64)std::llround((double)(c[0] - 1) / City::block_size);
        int64 by = (int64)std::llround((double)(c[1] - 1) / City::block_size);
        a.current = {bx * (int64)City::block_size + 1, by * (int64)City::block_size + 1};
        a.prev = a.current;
    }

    if (length(City::cell_center(*a.current) - Vec<2>{pos[0], pos[1]}) <= 2.0f * City::cell_size) {
        City::Pos old_current = *a.current;
        const int64 b = (int64)City::block_size;
        City::Pos nbrs[4] = {{old_current[0] + b, old_current[1]},
                             {old_current[0] - b, old_current[1]},
                             {old_current[0], old_current[1] + b},
                             {old_current[0], old_current[1] - b}};
        std::vector<City::Pos> choices;
        for (auto& n : nbrs)
            if (!a.prev || n != *a.prev) choices.push_back(n);
        City::Pos next = choices[hash(s.city.seed, s.tick, i) % choices.size()];
        a.prev = old_current;
        a.current = next;
    }
    auto target = City::cell_center(*a.current);
    return {target[0], target[1], ai_hover_altitude};
}
}  // namespace

struct AI::State {
    std::vector<DroneAI> drones;
};

AI::AI() : state(std::make_unique<State>()) {}
AI::~AI() = default;

void AI::update(GameState& s) {
    if (state->drones.size() != s.drones.size()) state->drones.resize(s.drones.size());

    foreach_drone(
        s,
        [&](size_t i, Drone& d) {
            if (d.status != Drone::Status::Alive) {
                state->drones[i].current = {};
                state->drones[i].prev = {};
            }
        },
        /*status*/ std::nullopt);

    foreach_drone(s, [&](size_t i, Drone& d) {
        if (i == 0) return;  // player drone
        auto& a = state->drones[i];

        Vec<3> wp = choose_next_waypoint(s, i, a, d.position);

        // Lift - hover at the waypoint altitude
        d.control_m[2] = a.lift(wp[2], d.position[2], Physics::dt);

        // Yaw - face the waypoint
        Vec<2> desired = normalise(Vec<2>{wp[0] - d.position[0], wp[1] - d.position[1]});
        Vec<3> fwd = rotate(d.orientation, {1.0f, 0.0f, 0.0f});
        Vec<2> fwd_xy = normalise(Vec<2>{fwd[0], fwd[1]});
        float yaw_err = std::atan2(fwd_xy[0] * desired[1] - fwd_xy[1] * desired[0],
                                   fwd_xy[0] * desired[0] + fwd_xy[1] * desired[1]);
        d.control_m[3] = a.yaw(0.0f, yaw_err, Physics::dt);

        // Pitch - aim for a target nose-down angle (when facing the waypoint)
        float target_pitch =
            std::clamp(1 - std::abs(yaw_err) / ai_yaw_deadband, 0.0f, 1.0f) * ai_pitch_angle;
        float pitch_angle = -std::asin(std::clamp(fwd[2], -1.0f, 1.0f));
        d.control_m[1] = a.pitch(target_pitch, pitch_angle, Physics::dt);

        // Lateral Vel > Roll - aim for zero lateral motion.
        float lat_vel = -d.velocity[0] * fwd_xy[1] + d.velocity[1] * fwd_xy[0];
        float target_roll = -a.lateral(0.0f, lat_vel, Physics::dt);
        Vec<3> right = rotate(d.orientation, {0.0f, 1.0f, 0.0f});
        float roll_angle = std::asin(std::clamp(right[2], -1.0f, 1.0f));
        d.control_m[0] = a.roll(target_roll, roll_angle, Physics::dt);
    });
}
