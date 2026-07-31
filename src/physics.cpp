#include "game.hpp"

namespace {
constexpr Vec<3> UP = {0.0f, 0.0f, 1.0f};
constexpr Vec<3> GRAVITY = {0.0f, 0.0f, -10.0f};
constexpr float MOTOR_FORCE = 5.0f;
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
    for (Drone& d : state.drones) {
        // Motor controls
        auto up = std::max(0.0f, d.control[2]);
        auto cFL = up;
        auto cFR = up;
        auto cBL = up;
        auto cBR = up;

        // Forces
        auto force = d.mass * GRAVITY;
        force += MOTOR_FORCE * (cFL + cFR + cBL + cBR) * rotate(d.orientation, UP);

        // Dynamics
        d.velocity += (dt / d.mass) * force;
        d.position += dt * d.velocity;

        // Collisions (floor)
        if (d.position[2] < 0.0f) {
            d.position[2] = 0.0f;
            if (d.velocity[2] < 0.0f) d.velocity[2] = 0.0f;
        }
    }
}
