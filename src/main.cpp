#include <iostream>
#include <fstream>
#include <cstdlib>

#include "PID.hpp"
#include "Motor.hpp"
#include "Drone.hpp"

int main(int, char**){
    // Output file setup
    std::ofstream file("output.csv");
    file << "time,target angle,angle,angular velocity,left RPM,right RPM\n";

    // Simulation parameters
    double dt = 0.001;
    double simulationTime = 30.0;

    // Motor initialization
    Motor motorFL(16.0, 4000.0, 1.8e-7);
    Motor motorFR(16.0, 4000.0, 1.8e-7);

    // Drone initialization
    Drone drone(0.25, 0.003, motorFL, motorFR);

    // Controller initialization
    PID pidYaw(3.0, 0.1, 0.5);

    double targetAngle = 0.0;
    double baseVoltage = 8.0;

    for (double t = 0.0; t < simulationTime; t +=dt)
    {
        // Angle
        if (t < 5.0) {
            targetAngle = 0.0;
        } else if (t < 10.0) {
            targetAngle = 0.5;
        } else if (t < 15.0) {
            targetAngle = 0.2;
        } else if (t < 20.0) {
            targetAngle = -0.5;
        } else if (t < 25.0) {
            targetAngle = 0.0;
        }

        double currentAngle = drone.getXAngle();
        double deltaVoltage = pidYaw.update(targetAngle, currentAngle, dt);

        double voltageFL = baseVoltage + deltaVoltage;
        double voltageFR = baseVoltage - deltaVoltage;

        drone.update(voltageFL, voltageFR, dt);

        file << t << ","
        << targetAngle << ","
        << drone.getXAngle() << ","
        << drone.getXAngularVelocity() << ","
        << motorFL.getRPM() << ","
        << motorFR.getRPM() << "\n";
    }

    system("python ../scripts/plot.py");

    return 0;
}
