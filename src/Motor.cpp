#include "Motor.hpp"
#include <algorithm>
#include <cmath>

Motor::Motor(double maxVoltage, double KV, double thrustCoefficient, double torqueCoefficient)
    : maxVoltage(maxVoltage), KV(KV), thrustCoefficient(thrustCoefficient), torqueCoefficient(torqueCoefficient)
{
}

double Motor::getThrust(double appliedVoltage) 
{
    double clampedVoltage = std::clamp(appliedVoltage, 0.0, maxVoltage);

    rpm = clampedVoltage * KV;

    double radPerSec = rpm * (2.0 * M_PI / 60.0);
    
    return pow(radPerSec, 2) * thrustCoefficient;
}

double Motor::getTorque(double appliedVoltage)
{
    double clampedVoltage = std::clamp(appliedVoltage, 0.0, maxVoltage);

    rpm = clampedVoltage * KV;

    double radPerSec = rpm * (2.0 * M_PI / 60.0);

    return pow(radPerSec, 2) * torqueCoefficient;
}