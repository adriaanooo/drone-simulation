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
    // Create vector of force in local reference frame
    Vector3 localForce{
        0.0,
        0.0, 
        thrust[0] + thrust[1] + thrust[2] + thrust[3]
    };

    // Rotate the force vector wrt to the global reference frame
    Quaternion localForceQuaternion{
        0.0,
        localForce.x,
        localForce.y,
        localForce.z
    };
    Quaternion globalForceQuarternion = quaternion * localForceQuaternion * quaternion.inverse();
    globalForce = {
        globalForceQuarternion.x,
        globalForceQuarternion.y,
        globalForceQuarternion.z - (9.81 * mass)
    };

    // Create a vector of moments in the local reference frame
    Vector3 localMoment = {
        (thrust[0] + thrust[2] - thrust[1] - thrust[3]) * momentArm,
        (thrust[0] + thrust[1] - thrust[2] - thrust[3]) * momentArm,
        torque[0] - torque[1] - torque[2] + torque[3]
    };

    // Compute angular states
    angularAcceleration = inertia.inverse() * (localMoment - (angularRate.crossProduct(inertia * angularRate)));
    angularRate = angularRate + angularAcceleration * dt;
    quaternion.applyAngularRate(angularRate, dt);

    // Compute translational states
    acceleration = globalForce * (1 / mass);
    velocity = velocity + acceleration * dt;
    position = position + velocity * dt;
}