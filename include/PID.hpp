#pragma once
#include <algorithm>
#include <cmath>

class PID
{
    public:
        PID(
            const double kp, 
            const double ki, 
            const double kd, 
            const double updateRate, 
            const double derivativeCutoffFreq);

        double update(
            const double setpoint, 
            const double measurement, 
            const double t, 
            const double dt);

    private:
        const double kp;
        const double ki;
        const double kd;
        const double updateRate;
        const double derivativeCutoffFreq;

        double integral{0.0};
        double previousError{0.0};
        double measurementFiltered{0.0};
        double previousMeasurementFiltered{0.0};
        double previousOutput{0.0};
        double timeSinceLastUpdate{0.0};
};