#include <iostream>
#include <fstream>
#include <cstdlib>

#include "PID.hpp"
#include "Motor.hpp"
#include "Drone.hpp"

int main(int, char**)
{
    // CSV header 
    std::ofstream file("output.csv");
    file << "Time (s),"
    << "Target Roll Angle (rad),"
    << "Target Pitch Angle (rad),"
    << "Target Yaw Angle (rad),"
    << "Roll Angle (rad),"
    << "Roll Velocity (rad/s),"
    << "Pitch Angle (rad),"
    << "Pitch Velocity (rad/s),"
    << "Yaw Angle (rad),"
    << "Yaw Velocity (rad/s),"
    << "FL Motor RPM,"
    << "FR Motor RPM,"
    << "RL Motor RPM,"
    << "RR Motor RPM"
    << "\n";

    // Simulation parameters
    double dt = 0.001;
    double simulationTime = 10.0;

    // Motor initialization
    Motor motorFL(16.0, 4000.0, 1.8e-7, 0.048);
    Motor motorFR(16.0, 4000.0, 1.8e-7, 0.048);
    Motor motorRL(16.0, 4000.0, 1.8e-7, 0.048);
    Motor motorRR(16.0, 4000.0, 1.8e-7, 0.048);

    // Drone initialization
    Drone drone(0.25, 0.003, 0.003, 0.006, motorFL, motorFR, motorRL, motorRR);

    // Controller initialization
    PID pidRoll(8.0, 0.1, 0.3);
    PID pidPitch(8.0, 0.1, 0.3);
    PID pidYaw(8.0, 0.1, 0.3);

    double targetRollAngle = 0.0;
    double targetPitchAngle = 0.0;
    double targetYawAngle = 0.0;
    double baseVoltage = 8.0;

    for (double t = 0.0; t < simulationTime; t +=dt)
    {
        // Target sequencing
        if (t < 2.0) 
        {
            targetRollAngle = 0.0;
            targetPitchAngle = 0.0;
            targetYawAngle = 0.0;
        } else if (t < 4.0) 
        {
            targetRollAngle = 0.5;
            targetPitchAngle = -0.5;
            targetYawAngle = 1.0;
        } else if (t < 6.0) 
        {
            targetRollAngle = 0.2;
            targetPitchAngle = 0.5;
            targetYawAngle = -1.0;
        } else if (t < 8.0) 
        {
            targetRollAngle = -0.5;
            targetPitchAngle = 0.2;
            targetYawAngle = 2.0;
        } else if (t < 10.0) 
        {
            targetRollAngle = 0.0;
            targetPitchAngle = 0.0;
            targetYawAngle = 0.0;
        }

        // Roll controller
        double currentRollAngle = drone.getRollAngle();
        double rollAngleError = pidRoll.update(targetRollAngle, currentRollAngle, dt);

        // Pitch controller
        double currentPitchAngle = drone.getPitchAngle();
        double pitchAngleError = pidPitch.update(targetPitchAngle, currentPitchAngle, dt);

        // Yaw controller
        double currentYawAngle = drone.getYawAngle();
        double yawAngleError = pidYaw.update(targetYawAngle, currentYawAngle, dt);

        // Voltage input
        double voltageFL = baseVoltage + rollAngleError + pitchAngleError + yawAngleError;
        double voltageFR = baseVoltage - rollAngleError + pitchAngleError - yawAngleError;
        double voltageRL = baseVoltage + rollAngleError - pitchAngleError - yawAngleError;
        double voltageRR = baseVoltage - rollAngleError - pitchAngleError + yawAngleError;

        // Drone state update
        drone.update(voltageFL, voltageFR, voltageRL, voltageRR, dt);

        // CSV output
        file << t << ","
        << targetRollAngle << ","
        << targetPitchAngle << ","
        << targetYawAngle << ","
        << drone.getRollAngle() << ","
        << drone.getRollVelocity() << ","
        << drone.getPitchAngle() << ","
        << drone.getPitchVelocity() << ","
        << drone.getYawAngle() << ","
        << drone.getYawVelocity() << ","
        << motorFL.getRPM() << ","
        << motorFR.getRPM() << ","
        << motorRL.getRPM() << ","
        << motorRR.getRPM() << "\n";
    }

    // Plotting
    system("python ../scripts/plot.py");

    return 0;
}
