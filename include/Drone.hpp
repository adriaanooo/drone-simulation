#pragma once
#include "Motor.hpp"

class Drone {

    public:
        Drone(double armLength, double inertiaXX, double inertiaYY, double inertiaZZ, Motor& motorFL, Motor& motorFR, Motor& motorRL, Motor& motorRR);

        void update(double voltageFL, double voltageFR, double voltageRL, double voltageRR, double dt);

        double getRollAngle() const { return rollAngle; }
        double getRollVelocity() const { return rollVelocity; }
        double getRollAcceleration() const { return rollAcceleration; }

        double getPitchAngle() const { return pitchAngle; }
        double getPitchVelocity() const { return pitchVelocity; }
        double getPitchAcceleration() const { return pitchAcceleration; }

        double getYawAngle() const { return yawAngle; }
        double getYawVelocity() const { return yawVelocity; }
        double getYawAcceleration() const { return yawAcceleration; }

    private:
        double armLength;
        double inertiaXX;
        double inertiaYY;
        double inertiaZZ;

        Motor& motorFL;
        Motor& motorFR;
        Motor& motorRL;
        Motor& motorRR;

        double rollAngle{0.0};
        double rollVelocity{0.0};
        double rollAcceleration{0.0};

        double pitchAngle{0.0};
        double pitchVelocity{0.0};
        double pitchAcceleration{0.0};

        double yawAngle{0.0};
        double yawVelocity{0.0};
        double yawAcceleration{0.0};
};