#include <iostream>
#include <fstream>
#include <cstdlib>
#include "PID.hpp"

int main(int, char**){
    // Output file setup
    std::ofstream file("output.csv");
    file << "time,target angle,angle,angular velocity,torque\n";

    // Simulation parameters
    double dt = 0.001;
    double simulationTime = 60.0;

    // Rigid body parameters
    double angle = 0.0;
    double angularVelocity = 0.0;
    double rotationalInertia = 0.01;

    // Controller
    PID pid(5, 0, 10);

    // Simulation
    double targetAngle = 0.5;

    for (double time = 0.0; time < simulationTime; time +=dt)
    {
        double torque = pid.update(targetAngle, angle, dt);

        // Torque limiter
        if (torque > 2) {
            torque = 2; 
        }

        double angularAcceleration = torque / rotationalInertia;
        
        angularVelocity += angularAcceleration * dt;
        angle += angularVelocity * dt;

        file << time << "," << targetAngle << "," << angle << "," << angularVelocity << "," << torque << "\n";
    }

    system("python ../scripts/plot.py");

    return 0;
}
