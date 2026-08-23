#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>

#include "PID.hpp"
#include "Motor.hpp"
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
    << "RR Motor RPM"
    << "\n";

    // Simulation parameters
    double dt = 0.00001;
    double simulationTime = 10.0;

    // Motor initialization
    Motor motorFL(16.0, 4000.0, 1.8e-7, 0.9e-7, 2.4e-4);
    Motor motorFR(16.0, 4000.0, 1.8e-7, 0.9e-7, 2.4e-4);
    Motor motorRL(16.0, 4000.0, 1.8e-7, 0.9e-7, 2.4e-4);
    Motor motorRR(16.0, 4000.0, 1.8e-7, 0.9e-7, 2.4e-4);

    // Drone initialization
    Drone drone(0.25, 0.003, 0.003, 0.006, motorFL, motorFR, motorRL, motorRR);

    // PID controller initialization
    PID pidRollRate(50.0, 0.0, 0.0);
    PID pidPitchRate(80.0, 0.0, 0.0);
    PID pidYawRate(80.0, 0.0, 0.0);

    // Command controller sequencing
    std::vector<Step> steps = {
        {0.0, 0.0, 0.0, 0.0},
        {1.0, 1.0, 0.0, 0.0},
        {2.0, 0.0, 1.0, 0.0},
        {3.0, 0.0, 0.0, 2.0},
        {4.0, 0.0, 0.0, 0.0},
        {5.0, -3.0, -3.0, -5.0},
        {6.0, 0.0, 0.0, 3.0},
        {7.0, 0.0, 2.0, 0.0},
        {8.0, 2.0, 0.0, 0.0},
        {9.0, 0.0, 0.0, 0.0},
    };

    StepController stepController(steps);

    double baseVoltage = 4.0;

    for (double t = 0.0; t < simulationTime; t +=dt)
    {
        // Command controller
        stepController.update(t);
        double targetRollRate = stepController.getRollRateTarget();
        double targetPitchRate = stepController.getPitchRateTarget();
        double targetYawRate = stepController.getYawRateTarget();

        // Roll controller
        double currentRollRate = drone.getRollRate();
        double rollRateDelta = pidRollRate.update(targetRollRate, currentRollRate, dt);

        // Pitch controller
        double currentPitchRate = drone.getPitchRate();
        double pitchRateDelta = pidPitchRate.update(targetPitchRate, currentPitchRate, dt);

        // Yaw controller
        double currentYawRate = drone.getYawRate();
        double yawRateDelta = pidYawRate.update(targetYawRate, currentYawRate, dt);

        // Voltage input
        double voltageFL = baseVoltage + rollRateDelta + pitchRateDelta + yawRateDelta;
        double voltageFR = baseVoltage - rollRateDelta + pitchRateDelta - yawRateDelta;
        double voltageRL = baseVoltage + rollRateDelta - pitchRateDelta - yawRateDelta;
        double voltageRR = baseVoltage - rollRateDelta - pitchRateDelta + yawRateDelta;

        // State update
        drone.update(voltageFL, voltageFR, voltageRL, voltageRR, dt);

        // CSV output
        file << t << ","
        << targetRollRate << ","
        << targetPitchRate << ","
        << targetYawRate << ","
        << drone.getRollAngleDeg() << ","
        << drone.getRollRateDeg() << ","
        << drone.getPitchAngleDeg() << ","
        << drone.getPitchRateDeg() << ","
        << drone.getYawAngleDeg() << ","
        << drone.getYawRateDeg() << ","
        << motorFL.getRPM() << ","
        << motorFR.getRPM() << ","
        << motorRL.getRPM() << ","
        << motorRR.getRPM() << "\n";
    }

    // Plotting
    file.close();
    system("python ../scripts/plot.py");

    return 0;
}
