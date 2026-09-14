#pragma once

#include "IMU.hpp"
#include "PID.hpp"
#include "ESC.hpp"
#include "Vector3.hpp"

class FlightController
{
    public:
        FlightController(
            const double updateRate,
            IMU& imu, 
            PID& rollRateController, 
            PID& pitchRateController,
            PID& yawRateController
        );

        void update(
            double baseMotorCommand, 
            const Vector3& angularRate,
            Vector3& angularRateTarget, 
            double dt
        );

        const Vector3& getPIDOutput() const { return motorCommandOffset; }

        double getESCCommandFL() const { return escCommandFL; }
        double getESCCommandFR() const { return escCommandFR; }
        double getESCCommandRL() const { return escCommandRL; }
        double getESCCommandRR() const { return escCommandRR; }

    private:
        double const updateRate;
        double const updatePeriod;
        IMU& imu; 
        PID& rollRateController; 
        PID& pitchRateController;
        PID& yawRateController;

        Vector3 motorCommandOffset{0.0, 0.0, 0.0};

        double escCommandFL{0.0};
        double escCommandFR{0.0};
        double escCommandRL{0.0};
        double escCommandRR{0.0};

        double t{0.0};
        double timeSinceLastUpdate{0.0};
};