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
    << "RR Motor RPM,"
    << "PID Roll,"
    << "PID Pitch,"
    << "PID Yaw"
    << "\n";

    // Simulation parameters
    double dt = 1.0e-5;
    double simulationTime = 5.0;

    // Motor initialization
    Motor motorFL(16.8, 5000.0, 2.3e-8, 3.5e-10, 1.0e-2);
    Motor motorFR(16.8, 5000.0, 2.3e-8, 3.5e-10, 1.0e-2);
    Motor motorRL(16.8, 5000.0, 2.3e-8, 3.5e-10, 1.0e-2);
    Motor motorRR(16.8, 5000.0, 2.3e-8, 3.5e-10, 1.0e-2);

    // Drone initialization
    Drone drone(0.220, 3.5e-4, 3.5e-4, 6.0e-4, motorFL, motorFR, motorRL, motorRR);

    // PID controller initialization
    PID pidRollRate(30.4, 1.0, 0.0266, 4000); // Ku = 38, Tu = 0.007
    PID pidPitchRate(30.4, 1.0, 0.0266, 4000); // Ku = 38, Tu = 0.007
    PID pidYawRate(10.0, 1.0, 0.0, 4000); // Ku = , Tu = 

    // Command controller sequencing
    std::vector<Step> steps = {
        {0.5, 5.0, 0.0, 0.0},
        {1.0, -5.0, 0.0, 0.0},
        {1.5, 0.0, 5.0, 0.0},
        {2.0, 0.0, -5.0, 0.0},
        {2.5, 0.0, 0.0, 10.0},
        {3.0, 0.0, 0.0, -10.0},
        {3.5, -5.0, -5.0, -10.0},
        {4.0, 5.0, 5.0, 10.0},
        {4.5, 0.0, 0.0, 0.0},
    };

    StepController stepController(steps);

    double baseVoltage = 10.0;

    for (double t = 0.0; t < simulationTime; t +=dt)
    {
        // Command controller
        stepController.update(t);
        double targetRollRate = stepController.getRollRateTarget();
        double targetPitchRate = stepController.getPitchRateTarget();
        double targetYawRate = stepController.getYawRateTarget();


        // Roll controller
        double currentRollRate = drone.getRollRate();
        double rollRateDelta = pidRollRate.update(targetRollRate, currentRollRate, t, dt, 6.9, -10);

        // Pitch controller
        double currentPitchRate = drone.getPitchRate();
        double pitchRateDelta = pidPitchRate.update(targetPitchRate, currentPitchRate, t, dt, 6.9, -10);

        // Yaw controller
        double currentYawRate = drone.getYawRate();
        double yawRateDelta = pidYawRate.update(targetYawRate, currentYawRate, t, dt, 6.9, -10);

        // Voltage input
        double voltageFL = baseVoltage + rollRateDelta + pitchRateDelta + yawRateDelta;
        double voltageFR = baseVoltage - rollRateDelta + pitchRateDelta - yawRateDelta;
        double voltageRL = baseVoltage + rollRateDelta - pitchRateDelta - yawRateDelta;
        double voltageRR = baseVoltage - rollRateDelta - pitchRateDelta + yawRateDelta;

        // State update
        drone.update(voltageFL, voltageFR, voltageRL, voltageRR, dt);

        // CSV output
        file << t << ","
        << targetRollRate * 180.0 / M_PI << ","
        << targetPitchRate * 180.0 / M_PI << ","
        << targetYawRate * 180.0 / M_PI << ","
        << drone.getRollAngleDeg() << ","
        << drone.getRollRateDeg() << ","
        << drone.getPitchAngleDeg() << ","
        << drone.getPitchRateDeg() << ","
        << drone.getYawAngleDeg() << ","
        << drone.getYawRateDeg() << ","
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
