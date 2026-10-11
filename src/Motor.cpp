#include "Motor.hpp"

Motor::Motor(
    const double KV, 
    const double thrustCoefficient, 
    const double torqueCoefficient, 
    const double timeConstant
)
    : KV(KV), 
    thrustCoefficient(thrustCoefficient), 
    torqueCoefficient(torqueCoefficient), 
    timeConstant(timeConstant)
{
}

void Motor::update(const double appliedVoltage, const double dt)
{
    // Get the commanded rotational velocity from the applied voltage and KV
    const double rpmCommand = appliedVoltage * KV;
    const double rotationalVelocityCommand = rpmCommand * 2 * M_PI / 60.0;

    // First-order motor model for calculating rotational acceleration
    rotationalAcceleration = (rotationalVelocityCommand - rotationalVelocity) / timeConstant;
    rotationalVelocity += rotationalAcceleration * dt;
}

double Motor::getThrust() 
{
    return pow(rotationalVelocity, 2) * thrustCoefficient;
}

double Motor::getTorque()
{
    return pow(rotationalVelocity, 2) * torqueCoefficient;
}