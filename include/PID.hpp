#pragma once

class PID
{
    public:
        PID(double kp, double ki, double kd, double updateRate);

        double update(double setpoint, double measurement, double t, double dt, double maxOutput, double minOutput);

        void reset();

    private:
        double kp;
        double ki;
        double kd;
        double updatePeriod;

        double integral{0.0};
        double previousError{0.0};
        double previousMeasurement{0.0};
        double previousOutput{0.0};
        double timeSinceLastUpdate{0.0};
};