#pragma once
#include "Drone.hpp"
#include "FlightController.hpp"
#include "Motor.hpp"
#include "PID.hpp"
#include "IMU.hpp"
#include "ESC.hpp"

class ControlSystem
{
    public:
        ControlSystem(
            const double dt,
            Drone plant,
            FlightController flightController,
            Motor motor,
            IMU imu,
            ESC esc,
            PID pidRoll,
            PID pidPitch,
            PID pidYaw
        );
        
    private:
        double t{0.0};
        const double dt;

        Drone plant;
        FlightController flightController;
        Motor motor;
        IMU imu;
        ESC esc;
};