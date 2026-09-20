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
            const Matrix3& inertia, 
            Motor& motorFL, 
            Motor& motorFR, 
            Motor& motorRL, 
            Motor& motorRR,
            ESC& escFL,
            ESC& escFR,
            ESC& escRL,
            ESC& escRR
        );

        void update(double escCommandFL, double escCommandFR, double escCommandRL, double escCommandRR, double dt);

        const double getMass() const { return mass; }

        const Vector3& getAngle() const { return angle; }
        const Vector3& getAngularRate() const { return angularRate; }
        const Vector3& getAngularAcceleration() const { return angularAcceleration; }

        const Vector3 getAngleDeg() const { return angle * RAD_TO_DEG; }
        const Vector3 getRateDeg() const { return angularRate * RAD_TO_DEG; }
        const Vector3 getAccelerationDeg() const { return angularAcceleration * RAD_TO_DEG; }

    private:
        const double mass;
        const double armLength;
        const double momentArm;
        const Matrix3& inertia;

        Motor& motorFL;
        Motor& motorFR;
        Motor& motorRL;
        Motor& motorRR;
        ESC& escFL;
        ESC& escFR;
        ESC& escRL;
        ESC& escRR;

        Vector3 localMoment{0.0, 0.0, 0.0};
        Vector3 localForce{0.0, 0.0, 0.0};
        Vector3 globalForce{0.0, 0.0, 0.0};

        Quaternion quaternion{};

        Vector3 angle{0.0, 0.0, 0.0};
        Vector3 angularRate{0.0, 0.0, 0.0};
        Vector3 angularAcceleration{0.0, 0.0, 0.0};

        Vector3 position{0.0, 0.0, 0.0};
        Vector3 velocity{0.0, 0.0, 0.0};
        Vector3 acceleration{0.0, 0.0, 0.0};
};