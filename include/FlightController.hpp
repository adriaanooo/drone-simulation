#pragma once

#include "IMU.hpp"
#include "PID.hpp"
#include "Vector3.hpp"
#include <array>
#include <algorithm>

class FlightController
{
    public:
        FlightController(
            const double updateRate,
            IMU imu, 
            std::array<PID, 3> rateControllers
        );

        void update(
            const double baseMotorCommand, 
            const Vector3 angularRates,
            const Vector3 angularRateTargets, 
            const double dt
        );

        Vector3 getPIDOutputs() const { return motorCommandOffsets; }
        std::array<double, 4> getESCCommands() const { return escCommands; }

    private:
        const double updateRate;
        IMU imu; 
        std::array<PID, 3> rateControllers;

        Vector3 motorCommandOffsets{0.0, 0.0, 0.0};
        std::array<double, 4> escCommands{0.0, 0.0, 0.0, 0.0};

        double t{0.0};
        double timeSinceLastUpdate{0.0};
};