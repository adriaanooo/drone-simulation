#include "Motor.hpp"
#include <algorithm>
#include <cmath>

Motor::Motor(double maxVoltage, double KV, double k)
    : maxVoltage(maxVoltage), KV(KV), k(k)
{
}

double Motor::getThrust(double appliedVoltage) {
    double clampedVoltage = std::clamp(appliedVoltage, 0.0, maxVoltage);

    rpm = clampedVoltage * KV;

    double radPerSec = rpm * (2.0 * M_PI / 60.0);
    
    return pow(radPerSec, 2) * k;
}