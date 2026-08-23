#include "Drone.hpp"
#include <cmath>

Drone::Drone(double armLength, double inertiaXX, double inertiaYY, Motor& motorFL, Motor& motorFR, Motor& motorRL, Motor& motorRR)
    : armLength(armLength), inertiaXX(inertiaXX), inertiaYY(inertiaYY), motorFL(motorFL), motorFR(motorFR), motorRL(motorRL), motorRR(motorRR)
{
}

void Drone::update(double voltageFL, double voltageFR, double voltageRL, double voltageRR, double dt) {
    double thrustFL = motorFL.getThrust(voltageFL);
    double thrustFR = motorFR.getThrust(voltageFR);
    double thrustRL = motorRL.getThrust(voltageRL);
    double thrustRR = motorRR.getThrust(voltageRR);

    // Roll
    double netRollTorque = (thrustFL + thrustRL - thrustFR - thrustRR) * armLength * sin(0.785398);

    rollAcceleration = netRollTorque / inertiaXX;
    rollVelocity += rollAcceleration * dt;
    rollAngle += rollVelocity * dt;

    // Pitch
    double netPitchTorque = (thrustFL + thrustFR - thrustRL - thrustRR) * armLength * sin(0.785398);

    pitchAcceleration = netPitchTorque / inertiaYY;
    pitchVelocity += pitchAcceleration * dt;
    pitchAngle += pitchVelocity * dt;
}