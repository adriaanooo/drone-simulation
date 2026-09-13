#pragma once
#include <cmath>

class Motor
{
    public:
        Motor(double KV, double thrustCoefficient, double torqueCoefficient, double timeConstant);

        void update(double appliedVoltage, double dt);

        double getKV() const { return KV; }
        double getThrustCoefficient() const { return thrustCoefficient; }

        double getThrust();
        double getTorque();
        double getRPM() const { return rotationalVelocity * 60.0 / (2 * M_PI); }

    private:
        double KV;
        double thrustCoefficient;
        double torqueCoefficient;
        double timeConstant;

        double rpm{0.0};
        double rotationalVelocity{0.0};
        double rotationalAcceleration{0.0};
};