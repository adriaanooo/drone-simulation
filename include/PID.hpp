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

        double integral;
        double previousError;
        double previousMeasurement;
        double previousOutput;
        double timeSinceLastUpdate;
};