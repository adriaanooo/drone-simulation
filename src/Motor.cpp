#include "Motor.hpp"
#include <algorithm>
#include <cmath>

Motor::Motor(
    double KV, 
    double thrustCoefficient, 
    double torqueCoefficient, 
    double timeConstant
)
    : KV(KV), 
    thrustCoefficient(thrustCoefficient), 
    torqueCoefficient(torqueCoefficient), 
    timeConstant(timeConstant)
{
}

void Motor::update(double appliedVoltage, double dt)
{
    double rpmCommand = appliedVoltage * KV;
    double rotationalVelocityCommand = rpmCommand * 2 * M_PI / 60.0;

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