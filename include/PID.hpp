#pragma once

class PID
{
    public:
        PID(double kp, double ki, double kd);

        double update(double setpoint, double measurement, double dt);

        void reset();

    private:
        double kp;
        double ki;
        double kd;

        double integral;
        double previousError;
};