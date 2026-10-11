#include "FlightController.hpp"

FlightController::FlightController(
    const double updateRate,
    IMU imu, 
    std::array<PID, 3> rateControllers
)
    : updateRate(updateRate),
    imu(imu),
    rateControllers(rateControllers)
{
}

void FlightController::update(
    const double baseMotorCommand, 
    const Vector3 angularRates,
    const Vector3 angularRateTargets, 
    const double dt
)
{
    timeSinceLastUpdate += dt;
    t += dt;

    if (timeSinceLastUpdate >= (1 / updateRate))
    {
        // IMU update
        imu.update(angularRates, (1 / updateRate));

        // PID updates
        motorCommandOffsets = {
            rateControllers[0].update(angularRateTargets.x, imu.getAngularRates().x, t, dt),
            rateControllers[1].update(angularRateTargets.y, imu.getAngularRates().y, t, dt),
            rateControllers[2].update(angularRateTargets.z, imu.getAngularRates().z, t, dt),
        };

        // Motor mixer
        escCommands = {
            std::clamp
            (
                baseMotorCommand + motorCommandOffsets.x + motorCommandOffsets.y + motorCommandOffsets.z,
                0.0,
                1.0
            ),
            std::clamp
            (
                baseMotorCommand - motorCommandOffsets.x + motorCommandOffsets.y - motorCommandOffsets.z,
                0.0,
                1.0
            ),
            std::clamp
            (
                baseMotorCommand + motorCommandOffsets.x - motorCommandOffsets.y - motorCommandOffsets.z,
                0.0,
                1.0
            ),
            std::clamp
            (
                baseMotorCommand - motorCommandOffsets.x - motorCommandOffsets.y + motorCommandOffsets.z,
                0.0,
                1.0
            )
        };

        // Reset update period counter
        timeSinceLastUpdate -= (1 / updateRate);
    }
}