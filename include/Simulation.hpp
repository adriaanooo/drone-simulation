#pragma once
#include "Drone.hpp"
#include "FlightController.hpp"
#include "Motor.hpp"
#include "PID.hpp"
#include "IMU.hpp"
#include "ESC.hpp"

class Simulation
{
    public:
        Simulation(
            const double dt,
            Drone drone,
            FlightController flightController,
            Motor motor,
            ESC esc
        );

    void update(const Vector3 angularRateTargets);

    // Getters
    DroneState getDroneState() const
    {
        return drone.getState();
    }
        
    private:
        double t{0.0};
        const double dt;

        Drone drone;
        FlightController flightController;
        std::array<Motor, 4> motors;
        std::array<ESC, 4> escs;
};