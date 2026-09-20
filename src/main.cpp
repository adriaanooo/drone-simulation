#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>

#include "Vector3.hpp"
#include "Matrix3.hpp"
#include "PID.hpp"
#include "Motor.hpp"
#include "IMU.hpp"
#include "Drone.hpp"
#include "Controller.hpp"
#include "FlightController.hpp"

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
    const double escUpdateRate = 400; // Hz
    const double flightControllerUpdateRate = 4000; // Hz
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

    // PID controller initialization
    PID rollRateController(0.04, 0.01, 0.0005, 400, derivativeCutoffFreq);
    PID pitchRateController(0.05, 0.01, 0.0005, 400, derivativeCutoffFreq);
    PID yawRateController(0.05, 0.01, 0.0, 400, derivativeCutoffFreq);

    // Flight controller initialization
    FlightController flightController(
        flightControllerUpdateRate,
        imu,
        rollRateController,
        pitchRateController,
        yawRateController
    );

    // Drone initialization
    const Matrix3 inertiaTensor = {{
        {0.000105, 0.0, 0.0},
        {0.0, 0.000132, 0.0},
        {0.0, 0.0, 0.000206}
    }};

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
        Vector3 angularRateTarget{
            stepController.getRollRateTarget(),
            stepController.getPitchRateTarget(),
            stepController.getYawRateTarget()
        };

        // Flight controller
        flightController.update(
            hoverThrottle,
            drone.getAngularRate(),
            angularRateTarget,
            dt
        );

        // State update
        drone.update(
            flightController.getESCCommandFL(),
            flightController.getESCCommandFR(),
            flightController.getESCCommandRL(),
            flightController.getESCCommandRR(),
            dt
        );

        // CSV output
        file << t << ","
        << angularRateTarget.x * 180.0 / M_PI << ","
        << angularRateTarget.y * 180.0 / M_PI << ","
        << angularRateTarget.z * 180.0 / M_PI << ","
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
        << flightController.getPIDOutput().x << ","
        << flightController.getPIDOutput().y << ","
        << flightController.getPIDOutput().z << ","
        << imu.getAngularRate().x << ","
        << imu.getAngularRate().y << ","
        << imu.getAngularRate().z << "\n";
    }

    // Plotting
    file.close();
    system("python ../scripts/plot.py");

    return 0;
}
