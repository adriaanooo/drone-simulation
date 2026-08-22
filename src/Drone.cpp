#include "Drone.hpp"

Drone::Drone(double armLength, double inertiaXX, Motor& motorFL, Motor& motorFR)
    : armLength(armLength), inertiaXX(inertiaXX), motorFL(motorFL), motorFR(motorFR)
{
}

void Drone::update(double voltageFL, double voltageFR, double dt) {
    double thrustFL = motorFL.getThrust(voltageFL);
    double thrustFR = motorFR.getThrust(voltageFR);

    double netTorque = (thrustFL - thrustFR) * armLength;

    xAngularAcceleration = netTorque / inertiaXX;
    xAngularVelocity += xAngularAcceleration * dt;
    xAngle += xAngularVelocity * dt;
}