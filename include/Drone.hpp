#pragma once

#include "Motor.hpp"
#include "IMU.hpp"
#include "Vector3.hpp"
#include "LowPassFilter.h"
#include <cmath>

static constexpr double RAD_TO_DEG = 180.0 / M_PI;

class Drone
{
    public:
        Drone(double armLength, const Vector3& inertia, Motor& motorFL, Motor& motorFR, Motor& motorRL, Motor& motorRR, IMU& imu, LowPassFilter& lowPassFilter);

        void update(double voltageFL, double voltageFR, double voltageRL, double voltageRR, double dt);

        const Vector3& getAngle() const { return angle; }
        const Vector3& getAngularRate() const { return angularRate; }
        const Vector3& getAngularAcceleration() const { return angularAcceleration; }

        Vector3 getAngleDeg() const { return angle * RAD_TO_DEG; }
        Vector3 getRateDeg() const { return angularRate * RAD_TO_DEG; }
        Vector3 getAccelerationDeg() const { return angularAcceleration * RAD_TO_DEG; }

    private:
        double armLength;
        const Vector3& inertia;

        Motor& motorFL;
        Motor& motorFR;
        Motor& motorRL;
        Motor& motorRR;
        IMU& imu;
        LowPassFilter& lowPassFilter;

        Vector3 angle{0.0, 0.0, 0.0};
        Vector3 angularRate{0.0, 0.0, 0.0};
        Vector3 angularAcceleration{0.0, 0.0, 0.0};
};