#include <algorithm>

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

// Drone control - PID settings
constexpr float max_rate = 2.5f;  // commanded pitch/roll rate, rad/s
constexpr float pid_kp = 5.0f;
constexpr float pid_ki = 3.0f;
constexpr float pid_kd = 0.0f;

constexpr float explosion_radius = 5.0f;
constexpr float lock_on_level = 80.0f;
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
}  // namespace

float Pid::operator()(float error, float dt) {
    integral += error * dt;
    integral = std::clamp(integral, -10.0f, 10.0f);  // anti-windup
    float delta = (error - prev) / dt;
    prev = error;
    return pid_kp * error + pid_ki * integral + pid_kd * delta;
}

Cell City::operator()(int64 x, int64 y) const {
    int64 ox = x - floordiv(x, int64(City::block_size)) * int64(City::block_size);
    int64 oy = y - floordiv(y, int64(City::block_size)) * int64(City::block_size);

    uint64 h = hash_mix(uint64(seed) ^ (uint64(x) * 0xb5297a4dULL ^ uint64(y) * 0x68e31da4ULL));

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
            float h = city(cx, cy).height;
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
    return d;
}

void Physics::tick(GameState& state) {
    ++state.tick;
    for (uint d_index = 0; d_index < state.drones.size(); ++d_index) {
        auto& d = state.drones[d_index];
        auto R = mat_from(d.orientation);
        auto Rt = transpose(R);
        auto ang_vel_m = mul(body_inv_inertia_m, mul(Rt, d.ang_momentum));  // actual rates (model)

        // Collisions (building)
        if (d.status != Drone::Status::Alive) continue;  // already dead
        if (hits_building(state.city, d.position, Drone::radius)) {
            d.status = Drone::Status::Crashed;
            continue;
        }

        // Alitutude-based lock-on death (player only)
        if (d_index == 0) {
            if (d.position[2] > lock_on_level) {
                d.lock_on += lock_on_rate * dt;
                if (d.lock_on >= 1.0f) {
                    d.status = Drone::Status::Crashed;
                    continue;
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

        // Motor controls
        auto lift = std::max(0.0f, d.control_m[2]);
        auto cFL = lift, cFR = lift, cBL = lift, cBR = lift;

        auto cw = d.control_m[3];  // (FL + BR) - (FR + BL)
        cFL += cw;
        cFR += -cw;
        cBL += -cw;
        cBR += cw;

        // Pitch/roll rates from a PID tracking the commanded rate (control in rad/s).
        auto rl = std::clamp(d.pid_roll(max_rate * d.control_m[0] - ang_vel_m[0], dt), -1.0f, 1.0f);
        auto fb =
            std::clamp(d.pid_pitch(max_rate * d.control_m[1] - ang_vel_m[1], dt), -1.0f, 1.0f);
        cFL += -fb + rl;
        cFR += -fb + -rl;
        cBL += fb + rl;
        cBR += fb + -rl;

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

        // if (d_index == 0) {
        //     print(state, 1000, "pos:", d.position[0], d.position[1], d.position[2]);
        // }
    }
}
