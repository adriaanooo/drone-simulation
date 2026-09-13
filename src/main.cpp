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
    << "PID Yaw,"
    << "IMU Roll Rate (deg/s),"
    << "IMU Pitch Rate (deg/s),"
    << "IMU Yaw Rate (deg/s)"
    << "\n";

    // Simulation parameters
    const double dt = 1.0e-5;
    const double simulationTime = 5.0;
    const double imuUpdateRate = 1000; // Hz
    const double escUpdateRate = 400; //Hz
    const double derivativeCutoffFreq = 20.0; // Hz

    // Motor initialization
    Motor motorFL(4000.0, 2.7e-7, 2.4e-9, 1.0e-2);
    Motor motorFR(4000.0, 2.7e-7, 2.4e-9, 1.0e-2);
    Motor motorRL(4000.0, 2.7e-7, 2.4e-9, 1.0e-2);
    Motor motorRR(4000.0, 2.7e-7, 2.4e-9, 1.0e-2);

    // ESC initialization
    ESC escFL(12.0, escUpdateRate);
    ESC escFR(12.0, escUpdateRate);
    ESC escRL(12.0, escUpdateRate);
    ESC escRR(12.0, escUpdateRate);

    // IMU initialization
    IMU imu(4.88e-4, imuUpdateRate); // 8.73e-4

    // Drone initialization
    const Vector3 inertiaTensor = {
        0.000105, // Ixx
        0.000132, // Iyy
        0.000206  // Izz
    };

    Drone drone(
        0.144, 
        0.075, 
        inertiaTensor, 
        motorFL,
        motorFR,
        motorRL, 
        motorRR,
        escFL,
        escFR,
        escRL,
        escRR
    );

    // PID controller initialization
    PID pidRollRate(0.01, 0.1, 0.00005, 400, derivativeCutoffFreq);
    PID pidPitchRate(0.01, 0.1, 0.00005, 400, derivativeCutoffFreq);
    PID pidYawRate(0.07, 0.1, 0.0, 400, derivativeCutoffFreq);

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

    double hoverThrottle = ((60 / (2 * M_PI * motorFL.getKV())) * sqrt((drone.getMass() * 9.81) / 
    (4 * motorFL.getThrustCoefficient()))) / escFL.getMaxVoltage();

    for (double t = 0.0; t < simulationTime; t +=dt)
    {
        // Command controller
        stepController.update(t);
        double targetRollRate = stepController.getRollRateTarget();
        double targetPitchRate = stepController.getPitchRateTarget();
        double targetYawRate = stepController.getYawRateTarget();

        // Roll controller
        double currentRollRate = imu.getAngularRate().x;
        double rollRateDelta = pidRollRate.update(targetRollRate, currentRollRate, t, dt);

        // Pitch controller
        double currentPitchRate = imu.getAngularRate().y;
        double pitchRateDelta = pidPitchRate.update(targetPitchRate, currentPitchRate, t, dt);

        // Yaw controller
        double currentYawRate = imu.getAngularRate().z;
        double yawRateDelta = pidYawRate.update(targetYawRate, currentYawRate, t, dt);

        // Motor mixer
        double escCommandFL = std::clamp(
            hoverThrottle + rollRateDelta + pitchRateDelta + yawRateDelta,
            0.0,
            1.0
        );
        double escCommandFR = std::clamp(
            hoverThrottle - rollRateDelta + pitchRateDelta - yawRateDelta,
            0.0,
            1.0
        );
        double escCommandRL = std::clamp(
            hoverThrottle + rollRateDelta - pitchRateDelta - yawRateDelta,
            0.0,
            1.0
        );
        double escCommandRR = std::clamp(
            hoverThrottle - rollRateDelta - pitchRateDelta + yawRateDelta,
            0.0,
            1.0
        );

        // State update
        drone.update(escCommandFL, escCommandFR, escCommandRL, escCommandRR, dt);
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
        << yawRateDelta << ","
        << imu.getAngularRate().x << ","
        << imu.getAngularRate().y << ","
        << imu.getAngularRate().z << "\n";
    }

    // Plotting
    file.close();
    system("python ../scripts/plot.py");

    return 0;
}
