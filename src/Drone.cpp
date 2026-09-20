#include "Drone.hpp"
#include <cmath>

Drone::Drone(
    double mass, 
    double armLength, 
    const Matrix3& inertia
)
    : mass(mass),
    armLength(armLength),
    momentArm(armLength * sin(45)),
    inertia(inertia)
{
}

void Drone::update(const double thrust[4], const double torque[4], double dt)
{
    Vector3 localForce{
        0.0,
        0.0, 
        thrust[0] + thrust[1] + thrust[2] + thrust[3]
    };

    Vector3 localMoment = {
        (thrust[0] + thrust[2] - thrust[1] - thrust[3]) * momentArm,
        (thrust[0] + thrust[1] - thrust[2] - thrust[3]) * momentArm,
        torque[0] - torque[1] - torque[2] + torque[3]
    };

    angularAcceleration = inertia.inverse() * (localMoment - (angularRate.crossProduct(inertia * angularRate)));
    angularRate = angularRate + angularAcceleration * dt;
    quaternion.applyAngularRate(angularRate, dt);
}