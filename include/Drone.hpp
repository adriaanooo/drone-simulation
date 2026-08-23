#pragma once
#include "Motor.hpp"

class Drone
{
    public:
        Drone(double armLength, double inertiaXX, double inertiaYY, double inertiaZZ, Motor& motorFL, Motor& motorFR, Motor& motorRL, Motor& motorRR);

        void update(double voltageFL, double voltageFR, double voltageRL, double voltageRR, double dt);

        double getRollAngle() const { return rollAngle; }
        double getRollRate() const { return rollRate; }
        double getRollAcceleration() const { return rollAcceleration; }

        double getPitchAngle() const { return pitchAngle; }
        double getPitchRate() const { return pitchRate; }
        double getPitchAcceleration() const { return pitchAcceleration; }

        double getYawAngle() const { return yawAngle; }
        double getYawRate() const { return yawRate; }
        double getYawAcceleration() const { return yawAcceleration; }

        double getRollAngleDeg() const { return rollAngle * 180.0 / M_PI; }
        double getRollRateDeg() const { return rollRate * 180.0 / M_PI; }
        double getRollAccelerationDeg() const { return rollAcceleration * 180.0 / M_PI; }

        double getPitchAngleDeg() const { return pitchAngle * 180.0 / M_PI; }
        double getPitchRateDeg() const { return pitchRate * 180.0 / M_PI; }
        double getPitchAccelerationDeg() const { return pitchAcceleration * 180.0 / M_PI; }

        double getYawAngleDeg() const { return yawAngle * 180.0 / M_PI; }
        double getYawRateDeg() const { return yawRate * 180.0 / M_PI; }
        double getYawAccelerationDeg() const { return yawAcceleration * 180.0 / M_PI; }

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
        double rollRate{0.0};
        double rollAcceleration{0.0};

        double pitchAngle{0.0};
        double pitchRate{0.0};
        double pitchAcceleration{0.0};

        double yawAngle{0.0};
        double yawRate{0.0};
        double yawAcceleration{0.0};
};