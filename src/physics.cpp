#include "game.hpp"

namespace {
constexpr Vec<3> UP = {0.0f, 0.0f, 1.0f};

// Physics
constexpr Vec<3> GRAVITY = {0.0f, 0.0f, -10.0f};
constexpr float MOTOR_FORCE = 5.0f;
constexpr float MOTOR_TORQUE = 0.3f;
constexpr float ANG_DAMPING = 0.4f;

// Control
constexpr auto C_ROT = 0.5f;
}  // namespace

Drone Drone::create(Vec<3> position) {
    Drone d;
    d.mass = 1.0f;
    d.inv_inertia = Mat<3, 3>{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 5.0f};
    d.position = position;
    d.velocity = {};
    d.orientation = {0.0f, 0.0f, 0.0f, 1.0f};
    d.ang_momentum = {};
    d.control = {};
    return d;
}

void Physics::tick(GameState& state) {
    ++state.tick;
    for (uint d_index = 0; d_index < state.drones.size(); ++d_index) {
        auto& d = state.drones[d_index];

        // Motor controls
        auto up = std::max(0.0f, d.control[2]);  // total
        auto cFL = up, cFR = up, cBL = up, cBR = up;

        auto cw = d.control[3];  // (FL + BR) - (FR + BL)
        cFL += C_ROT * cw;
        cFR -= C_ROT * cw;
        cBL -= C_ROT * cw;
        cBR += C_ROT * cw;

        // Forces
        auto force = d.mass * GRAVITY;
        force += MOTOR_FORCE * (cFL + cFR + cBL + cBR) * rotate(d.orientation, UP);

        // Torques
        auto torque = MOTOR_TORQUE * -(cFL - cFR - cBL + cBR) * UP;

        // Dynamics
        d.velocity += (dt / d.mass) * force;
        d.position += dt * d.velocity;

        auto model_to_world = mat_from(d.orientation);
        auto world_inv_inertia = mul(model_to_world, mul(d.inv_inertia, transpose(model_to_world)));
        auto ang_velocity = mul(world_inv_inertia, mul(d.ang_momentum, transpose(model_to_world)));
        d.ang_momentum += dt * (torque + (-ANG_DAMPING) * ang_velocity);
        d.orientation =
            normalise(d.orientation + 0.5 * dt * (quat_pure(ang_velocity) * d.orientation));

        // Collisions (floor)
        if (d.position[2] < 0.0f) {
            d.position[2] = 0.0f;
            if (d.velocity[2] < 0.0f) d.velocity[2] = 0.0f;
        }
    }
}
