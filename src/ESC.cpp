#include "ESC.hpp"

ESC::ESC(
    const double maxVoltage,
    const double updateRate
)
    : maxVoltage(maxVoltage),
    updateRate(updateRate)
{
}

void ESC::update(const double escCommand, const double dt)
{
    timeSinceLastUpdate += dt;

    if (timeSinceLastUpdate >= (1 / updateRate))
    {
        voltageOutput = escCommand * maxVoltage;
        timeSinceLastUpdate -= (1 / updateRate);
    }
}