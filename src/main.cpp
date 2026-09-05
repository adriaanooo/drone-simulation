#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>

#include "Vector3.hpp"
#include "PID.hpp"
#include "Motor.hpp"
#include "IMU.hpp"
#include "Drone.hpp"
#include "Controller.hpp"

int main(int, char**)
{
    // CSV header 
    std::ofstream file("output.csv");
    file << "Time (s),"
    << "Target Roll Rate (deg/s),"
    << "Target Pitch Rate (deg/s),"
    << "Target Yaw Rate (deg/s),"
    << "Roll Angle (deg),"
    << "Roll Rate (deg/s),"
    << "Pitch Angle (deg),"
    << "Pitch Rate (deg/s),"
    << "Yaw Angle (deg),"
    << "Yaw Rate (deg/s),"
    << "FL Motor RPM,"
    << "FR Motor RPM,"
    << "RL Motor RPM,"
    << "RR Motor RPM,"
    << "PID Roll,"
    << "PID Pitch,"
    << "PID Yaw"
    << "\n";

    // Simulation parameters
    constexpr double dt = 1.0e-5;
    constexpr double simulationTime = 5.0;
    constexpr double imuUpdateRate = 1000; // Hz
    constexpr double derivativeCutoffFreq = 20.0; // Hz

    // Motor initialization
    Motor motorFL(12.0, 4000.0, 2.7e-7, 2.4e-9, 1.0e-2);
    Motor motorFR(12.0, 4000.0, 2.7e-7, 2.4e-9, 1.0e-2);
    Motor motorRL(12.0, 4000.0, 2.7e-7, 2.4e-9, 1.0e-2);
    Motor motorRR(12.0, 4000.0, 2.7e-7, 2.4e-9, 1.0e-2);

    // IMU initialization
    IMU imu(4.88e-4, imuUpdateRate); // 8.73e-4

    // Drone initialization
    const Vector3 inertiaTensor = {0.000105, 0.000132, 0.000206};
    Drone drone(0.144, inertiaTensor, motorFL, motorFR, motorRL, motorRR);

    // PID controller initialization
    PID pidRollRate(0.2, 0.1, 0.001, 400, derivativeCutoffFreq); // Ku = 38, Tu = 0.007
    PID pidPitchRate(0.2, 0.1, 0.001, 400, derivativeCutoffFreq); // Ku = 38, Tu = 0.007
    PID pidYawRate(0.8, 0.1, 0.0, 400, derivativeCutoffFreq);

    // Command controller sequencing
    std::vector<Step> steps = {
        {0.5, 30.0, 0.0, 0.0},
        {1.0, -30.0, 0.0, 0.0},
        {1.5, 0.0, 30.0, 0.0},
        {2.0, 0.0, -30.0, 0.0},
        {2.5, 0.0, 0.0, 10.0},
        {3.0, 0.0, 0.0, -10.0},
        {3.5, -30.0, -30.0, -10.0},
        {4.0, 30.0, 30.0, 10.0},
        {4.5, 0.0, 0.0, 0.0},
    };

    StepController stepController(steps);

    double baseVoltage = 2.8;

    for (double t = 0.0; t < simulationTime; t +=dt)
    {
        // Command controller
        stepController.update(t);
        double targetRollRate = stepController.getRollRateTarget();
        double targetPitchRate = stepController.getPitchRateTarget();
        double targetYawRate = stepController.getYawRateTarget();

        // Roll controller
        double currentRollRate = imu.getAngularRate().x;
        double rollRateDelta = pidRollRate.update(targetRollRate, currentRollRate, t, dt, 9.2, -2.8);

        // Pitch controller
        double currentPitchRate = imu.getAngularRate().y;
        double pitchRateDelta = pidPitchRate.update(targetPitchRate, currentPitchRate, t, dt, 9.2, -2.8);

        // Yaw controller
        double currentYawRate = imu.getAngularRate().z;
        double yawRateDelta = pidYawRate.update(targetYawRate, currentYawRate, t, dt, 9.2, -2.8);

        // Motor mixer
        double voltageFL = baseVoltage + rollRateDelta + pitchRateDelta + yawRateDelta;
        double voltageFR = baseVoltage - rollRateDelta + pitchRateDelta - yawRateDelta;
        double voltageRL = baseVoltage + rollRateDelta - pitchRateDelta - yawRateDelta;
        double voltageRR = baseVoltage - rollRateDelta - pitchRateDelta + yawRateDelta;

        // State update
        drone.update(voltageFL, voltageFR, voltageRL, voltageRR, dt);
        imu.update(drone.getAngularRate(), dt);

        // CSV output
        file << t << ","
        << targetRollRate * 180.0 / M_PI << ","
        << targetPitchRate * 180.0 / M_PI << ","
        << targetYawRate * 180.0 / M_PI << ","
        << drone.getAngleDeg().x << ","
        << drone.getRateDeg().x << ","
        << drone.getAngleDeg().y << ","
        << drone.getRateDeg().y << ","
        << drone.getAngleDeg().z << ","
        << drone.getRateDeg().z << ","
        << motorFL.getRPM() << ","
        << motorFR.getRPM() << ","
        << motorRL.getRPM() << ","
        << motorRR.getRPM() << ","
        << rollRateDelta << ","
        << pitchRateDelta << ","
        << yawRateDelta << "\n";
    }

    // Plotting
    file.close();
    system("python ../scripts/plot.py");

    return 0;
}
