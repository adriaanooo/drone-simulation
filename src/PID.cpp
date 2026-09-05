#include "PID.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

PID::PID(double kp, double ki, double kd, double updateRate, double derivativeCutoffFreq): 
    kp(kp),
    ki(ki),
    kd(kd),
    updatePeriod(1.0 / updateRate),
    derivativeCutoffFreq(derivativeCutoffFreq)
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

        // Error calculation
        double error = setpoint - measurement;

        // Low pass filter for derivative controller
        double alpha = (1 / derivativeCutoffFreq) / ((1 / (2 * M_PI * derivativeCutoffFreq)) + (1 / derivativeCutoffFreq));
        measurementFiltered = previousMeasurementFiltered + alpha * (measurement - previousMeasurementFiltered);

        // Derivative calculation
        double derivative = -(measurementFiltered - previousMeasurementFiltered) / dtControl;

        // PID Outputs
        double proportionalOutput = kp * error;
        double integralOutput = ki * integral;
        double derivativeOutput = kd * derivative;

        // Output clamping to prevent integral windup
        double output = std::clamp(proportionalOutput + integralOutput + derivativeOutput, minOutput, maxOutput);

        // Integral calculation
        integral += error * dtControl;

        // Update previous values
        previousMeasurementFiltered = measurementFiltered;
        previousOutput = output;

        // Reset update timer
        timeSinceLastUpdate = 0.0;

        return output;
    }
    else
    {
        return previousOutput;
    }
}
