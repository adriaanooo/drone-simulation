#include "Drone.hpp"

Drone::Drone(
    const double mass, 
    const double armLength, 
    const Matrix3 inertiaTensor
)
    : mass(mass),
    armLength(armLength),
    momentArm(armLength * sin(45 * M_PI / 180.0)),
    inertiaTensor(inertiaTensor)
{
}

void Drone::update(
        const std::array<double, 4> motorThrusts, 
        const std::array<double, 4> motorTorques, 
        const double dt
    )
{
    // Create vector of force in local reference frame
    Vector3 localForces{
        0.0,
        0.0, 
        motorThrusts[0] + motorThrusts[1] + motorThrusts[2] + motorThrusts[3]
    };

    // Rotate the force vector wrt to the global reference frame using drone quaternion
    Vector3 globalForces = localForces.rotate(quaternion);
    globalForces.z = globalForces.z - (9.81 * mass); // Gravity

    // Create a vector of moments in the local reference frame
    Vector3 localMoments = {
        (motorThrusts[0] + motorThrusts[2] - motorThrusts[1] - motorThrusts[3]) * momentArm,
        (motorThrusts[0] + motorThrusts[1] - motorThrusts[2] - motorThrusts[3]) * momentArm,
        motorTorques[0] - motorTorques[1] - motorTorques[2] + motorTorques[3]
    };

    // Compute angular states
    angularAccelerations = inertiaTensor.inverse() * (localMoments - (angularRates.crossProduct(inertiaTensor * angularRates)));
    angularRates = angularRates + angularAccelerations * dt;
    quaternion.applyAngularRate(angularRates, dt);

    // Compute translational states
    accelerations = globalForces * (1 / mass);
    velocities = velocities + accelerations * dt;
    worldCoordinates = worldCoordinates + velocities * dt;
}