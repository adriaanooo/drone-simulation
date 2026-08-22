#pragma once
#include "Motor.hpp"

class Drone {

    public:
        Drone(double armLength, double inertiaXX, Motor& motorFL, Motor& motorFR);

        void update(double voltageFL, double voltageFR, double dt);

        double getXAngle() const { return xAngle; }
        double getXAngularVelocity() const { return xAngularVelocity; }
        double getXAngularAcceleration() const { return xAngularAcceleration; }

    private:
        double armLength;
        double inertiaXX;

        Motor& motorFL;
        Motor& motorFR;

        double xAngle{0.0};
        double xAngularVelocity{0.0};
        double xAngularAcceleration{0.0};
};