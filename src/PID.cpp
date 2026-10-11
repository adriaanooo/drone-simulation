#include "PID.hpp"

PID::PID(
    const double kp, 
    const double ki, 
    const double kd, 
    const double updateRate, 
    const double derivativeCutoffFreq
)
    : kp(kp),
    ki(ki),
    kd(kd),
    updateRate(updateRate),
    derivativeCutoffFreq(derivativeCutoffFreq)
{
}

double PID::update(double setpoint, double measurement, double t, double dt)
{
    timeSinceLastUpdate += dt;

    // Update PID controller if simulation timestep aligns with controller update rate
    if (timeSinceLastUpdate >= (1 / updateRate))
    {
        // This sets the time step of the controller to the time since the last update
        const double dtControl = timeSinceLastUpdate;

        // Error calculation
        const double error = setpoint - measurement;

        // Low pass filter for derivative controller
        const double alpha = (1 / derivativeCutoffFreq) / ((1 / (2 * M_PI * derivativeCutoffFreq)) + (1 / derivativeCutoffFreq));
        measurementFiltered = previousMeasurementFiltered + alpha * (measurement - previousMeasurementFiltered);

        // Derivative calculation
        const double derivative = -(measurementFiltered - previousMeasurementFiltered) / dtControl;

        // PID Outputs
        const double proportionalOutput = kp * error;
        const double integralOutput = ki * integral;
        const double derivativeOutput = kd * derivative;

        // Output clamping to prevent integral windup
        const double output = std::clamp(proportionalOutput + integralOutput + derivativeOutput, -1.0, 1.0);

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
