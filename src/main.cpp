#include <iostream>
#include <fstream>
#include <cstdlib>

#include "PID.hpp"
#include "Motor.hpp"
#include "Drone.hpp"

int main(int, char**)
{
    // Output file setup
    std::ofstream file("output.csv");
    file << "Time (s),"
    << "Target Roll Angle (rad),"
    << "Target Pitch Angle (rad),"
    << "Roll Angle (rad),"
    << "Roll Velocity (rad/s),"
    << "Pitch Angle (rad),"
    << "Pitch Velocity (rad/s),"
    << "FL Motor RPM,"
    << "FR Motor RPM,"
    << "RL Motor RPM,"
    << "RR Motor RPM"
    << "\n";

    // Simulation parameters
    double dt = 0.001;
    double simulationTime = 30.0;

    // Motor initialization
    Motor motorFL(16.0, 4000.0, 1.8e-7);
    Motor motorFR(16.0, 4000.0, 1.8e-7);
    Motor motorRL(16.0, 4000.0, 1.8e-7);
    Motor motorRR(16.0, 4000.0, 1.8e-7);

    // Drone initialization
    Drone drone(0.25, 0.003, 0.003, motorFL, motorFR, motorRL, motorRR);

    // Controller initialization
    PID pidRoll(5.0, 0.1, 0.2);
    PID pidPitch(5.0, 0.1, 0.2);

    double targetRollAngle = 0.0;
    double targetPitchAngle = 0.0;
    double baseVoltage = 8.0;

    for (double t = 0.0; t < simulationTime; t +=dt)
    {
        // Angle
        if (t < 5.0) 
        {
            targetRollAngle = 0.0;
            targetPitchAngle = 0.0;
        } else if (t < 10.0) 
        {
            targetRollAngle = 0.5;
            targetPitchAngle = -0.5;
        } else if (t < 15.0) 
        {
            targetRollAngle = 0.2;
            targetPitchAngle = 0.5;
        } else if (t < 20.0) 
        {
            targetRollAngle = -0.5;
            targetPitchAngle = 0.2;
        } else if (t < 25.0) 
        {
            targetRollAngle = 0.0;
            targetPitchAngle = 0.0;
        }

        // Roll PID controller
        double currentRollAngle = drone.getRollAngle();
        double rollAngleError = pidRoll.update(targetRollAngle, currentRollAngle, dt);

        // Pitch PID controller
        double currentPitchAngle = drone.getPitchAngle();
        double pitchAngleError = pidPitch.update(targetPitchAngle, currentPitchAngle, dt);

        // Voltage control
        double voltageFL = baseVoltage + rollAngleError + pitchAngleError;
        double voltageFR = baseVoltage - rollAngleError + pitchAngleError;
        double voltageRL = baseVoltage + rollAngleError - pitchAngleError;
        double voltageRR = baseVoltage - rollAngleError - pitchAngleError;

        drone.update(voltageFL, voltageFR, voltageRL, voltageRR, dt);

        file << t << ","
        << targetRollAngle << ","
        << targetPitchAngle << ","
        << drone.getRollAngle() << ","
        << drone.getRollVelocity() << ","
        << drone.getPitchAngle() << ","
        << drone.getPitchVelocity() << ","
        << motorFL.getRPM() << ","
        << motorFR.getRPM() << ","
        << motorRL.getRPM() << ","
        << motorRR.getRPM() << "\n";
    }

    system("python ../scripts/plot.py");

    return 0;
}
