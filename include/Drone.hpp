#pragma once

#include "Vector3.hpp"
#include "Matrix3.hpp"
#include "Quaternion.hpp"
#include <array>
#include <cmath>

struct DroneState
{
    Vector3 worldCoordinates;
    Vector3 worldVelocities;
    Vector3 worldAccelerations;
    Vector3 localAngularRates;
    Vector3 localAngularAccelerations;
    Quaternion orientation;
};

class Drone
{
    public:
        Drone(
            const double mass, 
            const double armLength, 
            const Matrix3 inertiaTensorValues
        );

        void update(
            const std::array<double, 4> motorThrusts, 
            const std::array<double, 4> motorTorques, 
            const double dt
        );

        // Getters
        Vector3 getAngularRates() const { return angularRates; }
        Vector3 getWorldCoordinates() const { return worldCoordinates; }
        DroneState getState() const
        {
            return {
                worldCoordinates,
                velocities,
                accelerations,
                angularRates,
                angularAccelerations,
                quaternion
            };
        }

    private:
        // Drone parameters
        const double mass;
        const double armLength;
        const double momentArm;
        const Matrix3 inertiaTensor;

        // Drone forces and moments
        Vector3 localMoments{0.0, 0.0, 0.0};
        Vector3 localForces{0.0, 0.0, 0.0};
        Vector3 globalForces{0.0, 0.0, 0.0};

        // Drone angular states
        Vector3 angularRates{0.0, 0.0, 0.0};
        Vector3 angularAccelerations{0.0, 0.0, 0.0};

        // Drone translational states
        Vector3 worldCoordinates{0.0, 0.0, 0.0};
        Vector3 velocities{0.0, 0.0, 0.0};
        Vector3 accelerations{0.0, 0.0, 0.0};

        Quaternion quaternion{};
};