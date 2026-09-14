#include "FlightController.hpp"

FlightController::FlightController(
    const double updateRate,
    IMU& imu, 
    PID& rollRateController, 
    PID& pitchRateController,
    PID& yawRateController
)
    : updateRate(updateRate),
    updatePeriod(1 / updateRate),
    imu(imu),
    rollRateController(rollRateController),
    pitchRateController(pitchRateController),
    yawRateController(yawRateController)
{
}

void FlightController::update(
    double baseMotorCommand, 
    const Vector3& angularRate,
    Vector3& angularRateTarget, 
    double dt
)
{
    timeSinceLastUpdate += dt;
    t += dt;

    if (timeSinceLastUpdate >= updatePeriod)
    {
        // IMU update
        imu.update(angularRate, updatePeriod);

        // PID updates
        motorCommandOffset.x = rollRateController.update(
            angularRateTarget.x, 
            imu.getAngularRate().x, 
            t, 
            updatePeriod
        );
        motorCommandOffset.y = pitchRateController.update(
            angularRateTarget.y, 
            imu.getAngularRate().y, 
            t, 
            updatePeriod
        );
        motorCommandOffset.z = yawRateController.update(
            angularRateTarget.z,
            imu.getAngularRate().z,
            t,
            updatePeriod
        );

        // Motor mixer
        escCommandFL = std::clamp(
            baseMotorCommand + motorCommandOffset.x + motorCommandOffset.y + motorCommandOffset.z,
            0.0,
            1.0
        );
        escCommandFR = std::clamp(
            baseMotorCommand - motorCommandOffset.x + motorCommandOffset.y - motorCommandOffset.z,
            0.0,
            1.0
        );
        escCommandRL = std::clamp(
            baseMotorCommand + motorCommandOffset.x - motorCommandOffset.y - motorCommandOffset.z,
            0.0,
            1.0
        );
        escCommandRR = std::clamp(
            baseMotorCommand - motorCommandOffset.x - motorCommandOffset.y + motorCommandOffset.z,
            0.0,
            1.0
        );

        // Reset update period counter
        timeSinceLastUpdate -= updatePeriod;
    }
}