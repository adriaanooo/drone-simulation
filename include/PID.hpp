#pragma once

class PID
{
    public:
        PID(double kp, double ki, double kd, double updateRate, const double derivativeCutoffFreq);

        double update(double setpoint, double measurement, double t, double dt, double maxOutput, double minOutput);

    private:
        double kp;
        double ki;
        double kd;
        double updatePeriod;
        double derivativeCutoffFreq;

        double integral{0.0};
        double previousError{0.0};
        double measurementFiltered{0.0};
        double previousMeasurementFiltered{0.0};
        double previousOutput{0.0};
        double timeSinceLastUpdate{0.0};
};