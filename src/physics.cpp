#include "game.hpp"

namespace {
constexpr Vec<3> UP = {0.0f, 0.0f, 1.0f};
constexpr Vec<3> GRAVITY = {0.0f, 0.0f, -10.0f};

// Drone
constexpr float BODY_MASS = 1.0f;
constexpr Mat3 DAMPING_M = mat_diag(0.2f, 0.2f, 0.4f);
constexpr Mat3 BODY_INV_INERTIA_M = mat_diag(1.0f, 1.0f, 5.0f);
constexpr float MOTOR_FORCE = 4.0f;
constexpr std::array<Vec<3>, 4> MOTOR_OFFSETS_M = {{
    {{0.1f, 0.1f, 0.0f}},    // FL
    {{0.1f, -0.1f, 0.0f}},   // FR
    {{-0.1f, 0.1f, 0.0f}},   // BL
    {{-0.1f, -0.1f, 0.0f}},  // BR
}};
constexpr float MOTOR_TORQUE = 0.3f;
constexpr Mat3 ANG_DAMPING_M = mat_diag(0.4f, 0.4f, 0.4f);

// Control
constexpr auto C_FB = 0.5f;
constexpr auto C_RL = 0.5f;
constexpr auto C_ROT = 0.5f;
}  // namespace

Drone Drone::create(Vec<3> position) {
    Drone d;
    d.position = position;
    d.velocity = {};
    d.orientation = {0.0f, 0.0f, 0.0f, 1.0f};
    d.ang_momentum = {};
    d.control_m = {};
    return d;
}

void Physics::tick(GameState& state) {
    ++state.tick;
    for (uint d_index = 0; d_index < state.drones.size(); ++d_index) {
        auto& d = state.drones[d_index];
        auto R = mat_from(d.orientation);
        auto Rt = transpose(R);

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
        auto up = std::max(0.0f, d.control_m[2]);  // total
        auto cFL = up, cFR = up, cBL = up, cBR = up;

        auto cw = d.control_m[3];  // (FL + BR) - (FR + BL)
        cFL += C_ROT * cw;
        cFR += C_ROT * -cw;
        cBL += C_ROT * -cw;
        cBR += C_ROT * cw;

        auto rl = d.control_m[0];
        auto fb = d.control_m[1];
        cFL += C_FB * -fb + C_RL * rl;
        cFR += C_FB * -fb + C_RL * -rl;
        cBL += C_FB * fb + C_RL * rl;
        cBR += C_FB * fb + C_RL * -rl;

        // Forces
        auto force = BODY_MASS * GRAVITY;
        force += MOTOR_FORCE * (cFL + cFR + cBL + cBR) * mul(R, UP);
        force -= mul(R, mul(DAMPING_M, mul(Rt, d.velocity)));

        // Torques
        auto torque_m = MOTOR_TORQUE * (-cFL + cFR + cBL - cBR) * UP;
        torque_m += cross(MOTOR_OFFSETS_M[0], cFL * MOTOR_FORCE * UP);
        torque_m += cross(MOTOR_OFFSETS_M[1], cFR * MOTOR_FORCE * UP);
        torque_m += cross(MOTOR_OFFSETS_M[2], cBL * MOTOR_FORCE * UP);
        torque_m += cross(MOTOR_OFFSETS_M[3], cBR * MOTOR_FORCE * UP);
        auto torque = mul(R, torque_m);

        // Dynamics
        d.velocity += (dt / BODY_MASS) * force;
        d.position += dt * d.velocity;
        auto ang_vel_m = mul(BODY_INV_INERTIA_M, mul(Rt, d.ang_momentum));
        auto ang_vel = mul(R, ang_vel_m);
        d.ang_momentum += dt * (torque - mul(R, mul(ANG_DAMPING_M, ang_vel_m)));
        d.orientation = normalise(d.orientation + 0.5 * dt * (quat_pure(ang_vel) * d.orientation));
    }
}
