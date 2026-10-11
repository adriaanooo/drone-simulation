#include <iostream>
#include <fstream>
#include <cstdlib>

#include "PID.hpp"
#include "Motor.hpp"
#include "IMU.hpp"
#include "ESC.hpp"
#include "Drone.hpp"
#include "FlightController.hpp"
#include "Simulation.hpp"

int main(int, char**)
{
    // Simulation parameters
    const double dt = 1.0e-5;
    const double simulationTime = 1.0;

    // Motor initialization
    Motor motor(4000.0, 2.7e-7, 2.4e-9, 1.0e-2);

    // ESC initialization
    ESC esc(12.0, 400);

    // IMU initialization
    IMU imu(4.88e-4, 1000); // 8.73e-4

    // PID controller initialization
    const std::array<PID, 3> rateControllers {
        PID(0.04, 0.3, 0.0005, 400, 20.0),      // Roll
        PID(0.03, 0.3, 0.0005, 400, 20.0),      // Pitch
        PID(0.05, 0.4, 0.0, 400, 20.0)          // Yaw
    };

    // Flight controller initialization
    FlightController flightController(
        4000.0,
        imu,
        rateControllers
    );

    // Drone initialization
    const Matrix3 inertiaTensor = {{
        {0.000105, 0.0, 0.0},
        {0.0, 0.000132, 0.0},
        {0.0, 0.0, 0.000206}
    }};

    Drone drone(0.144, 0.075, inertiaTensor);

    // Simulation initialization
    Simulation simulation(
        dt,
        drone,
        flightController,
        motor,
        esc
    );

    const Vector3 angularRateTargets = {5.0, 5.0, 10.0};

    for (double t = 0.0; t < simulationTime; t += dt)
    {
        // Update the simulation
        simulation.update(angularRateTargets);

        // Retrieve the drone state
        const DroneState droneState = simulation.getDroneState();
        
        // Print Output
        std::cout << "TIME: " << t << "\n";
        std::cout << "Roll Rate: " << droneState.localAngularRates.x << "\n";
        std::cout << "Pitch Rate: " << droneState.localAngularRates.y << "\n";
        std::cout << "Yaw Rate: " << droneState.localAngularRates.z << "\n";
    }

    return 0;
}
