#include "PID.hpp"
#include <algorithm>

PID::PID(double kp, double ki, double kd, double updateRate): 
    kp(kp),
    ki(ki),
    kd(kd),
    updatePeriod(1.0 / updateRate),
    integral(0.0),
    previousError(0.0),
    previousMeasurement(0.0),
    previousOutput(0.0),
    timeSinceLastUpdate(0.0)
{
}

double PID::update(double setpoint, double measurement, double t, double dt, double maxOutput, double minOutput)
{
    timeSinceLastUpdate += dt;

    // Update PID controller if simulation timestep aligns with controller update rate
    if (timeSinceLastUpdate >= updatePeriod)
    {
        // This sets the time step of the controller to the time since the last update
        double dtControl = timeSinceLastUpdate;

        double error = setpoint - measurement;
        double derivative = -(measurement - previousMeasurement) / dtControl;

        double proportionalOutput = kp * error;
        double integralOutput = ki * integral;
        double derivativeOutput = kd * derivative;

        double output = std::clamp(proportionalOutput + integralOutput + derivativeOutput, minOutput, maxOutput);

        integral += error * dt;
        previousError = error;
        previousMeasurement = measurement;
        previousOutput = output;

        timeSinceLastUpdate = 0.0;

        return output;
    }
    else
    {
        return previousOutput;
    }
}

void PID::reset()
{
    integral = 0.0;
    previousError = 0.0;
}