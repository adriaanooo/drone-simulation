#include "ControlSystem.hpp"

ControlSystem::ControlSystem(
    const double dt,
    Drone plant,
    FlightController flightController,
    Motor motor,
    IMU imu,
    ESC esc,
    PID pidRoll,
    PID pidPitch,
    PID pidYaw
)
    : dt(dt),
    plant(plant),
    flightController(flightController),
    motor(motor),
    imu(imu),
    esc(esc),
    pidRoll(pidRoll),
    pidPitch(pidPitch),
    pidYaw(pidYaw)
{}