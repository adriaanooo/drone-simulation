#include "ESC.hpp"

ESC::ESC(
    const double maxVoltage,
    const double updateRate
)
    : maxVoltage(maxVoltage),
    updatePeriod(1 / updateRate)
{
}

void ESC::update(double throttleInput, double dt)
{
    timeSinceLastUpdate += dt;

    if (timeSinceLastUpdate >= updatePeriod)
    {
        voltageOutput = throttleInput * maxVoltage;
        timeSinceLastUpdate = 0.0;
    }
}