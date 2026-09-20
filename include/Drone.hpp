#pragma once

#include "Motor.hpp"
#include "IMU.hpp"
#include "FlightController.hpp"
#include "Vector3.hpp"
#include "Matrix3.hpp"
#include "Quaternion.hpp"
#include <cmath>

static constexpr double RAD_TO_DEG = 180.0 / M_PI;
static constexpr double DEG_TO_RAD = M_PI / 180;

class Drone
{
    public:
        Drone(
            double mass, 
            double armLength, 
            const Matrix3& inertia 
        );

        void update(const double thrust[4], const double torque[4], const double dt);

        const double getMass() const { return mass; }

        const Vector3& getAngularRate() const { return angularRate; }
        const Vector3& getAngularAcceleration() const { return angularAcceleration; }

        const Vector3 getRateDeg() const { return angularRate * RAD_TO_DEG; }
        const Vector3 getAccelerationDeg() const { return angularAcceleration * RAD_TO_DEG; }

        const Vector3& getPosition() const { return position; }

    private:
        const double mass;
        const double armLength;
        const double momentArm;
        const Matrix3& inertia;

        Vector3 localMoment{0.0, 0.0, 0.0};
        Vector3 localForce{0.0, 0.0, 0.0};
        Vector3 globalForce{0.0, 0.0, 0.0};

        Quaternion quaternion{};

        Vector3 angularRate{0.0, 0.0, 0.0};
        Vector3 angularAcceleration{0.0, 0.0, 0.0};

        Vector3 position{0.0, 0.0, 0.0};
        Vector3 velocity{0.0, 0.0, 0.0};
        Vector3 acceleration{0.0, 0.0, 0.0};
};