#include "PID.hpp"

PID::PID(double kp, double ki, double kd): 
    kp(kp),
    ki(ki),
    kd(kd),
    integral(0.0),
    previousError(0.0)
{
}

double PID::update(double setpoint, double measurement, double dt)
{
    double error = setpoint - measurement;

    integral += error * dt;
    
    double derivative = (error - previousError) / dt;

    previousError = error;

    return kp * error + ki * integral + kd * derivative;
}

void PID::reset()
{
    integral = 0.0;
    previousError = 0.0;
}