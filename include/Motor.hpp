#pragma once
#include <algorithm>
#include <cmath>

class Motor
{
    public:
        Motor(
            const double KV, 
            const double thrustCoefficient, 
            const double torqueCoefficient, 
            const double timeConstant
        );

        void update(
            const double appliedVoltage, 
            const double dt
        );

        // Getters
        double getThrust();
        double getTorque();
        double getRPM() const { return rotationalVelocity * 60.0 / (2 * M_PI); }

    private:
        const double KV;
        const double thrustCoefficient;
        const double torqueCoefficient;
        const double timeConstant;

        double rpm{0.0};
        double rotationalVelocity{0.0};
        double rotationalAcceleration{0.0};
};