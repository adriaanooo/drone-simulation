#include "Simulation.hpp"

Simulation::Simulation(
    const double dt,
    Drone drone,
    FlightController flightController,
    Motor motor,
    ESC esc
)
    : dt(dt),
    drone(drone),
    flightController(flightController),
    motors{motor, motor, motor, motor},
    escs{esc, esc, esc, esc}
{}

void Simulation::update(Vector3 angularRateTargets)
{
    // Update the flight controller
    flightController.update(
        0.5,
        drone.getAngularRates(),
        angularRateTargets,
        dt
    );

    // Pass the flight controller's ESC commands to each esc
    for (int i = 0; i < 4; i++)
    {
        escs[i].update
        (
            flightController.getESCCommands()[i], dt
        );
    }

    // Pass the ESC voltage outputs to each motor
    for (int i = 0; i < 4; i++)
    {
        motors[i].update
        (
            escs[i].getVoltageOutput(), dt
        );
    }

    // Create motor thrust and torque arrays to pass to drone body
    const std::array<double, 4> motorThrusts = 
    {
        motors[0].getThrust(), motors[1].getThrust(), motors[2].getThrust(), motors[3].getThrust()
    };

    const std::array<double, 4> motorTorques = 
    {
        motors[0].getTorque(), motors[1].getTorque(), motors[2].getTorque(), motors[3].getTorque()
    };

    // Pass motor torque and thrust arrays to drone body to update drone state
    drone.update(motorThrusts, motorTorques, dt);
}
