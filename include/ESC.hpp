#pragma once
#include <array>

class ESC
{
    public:
        ESC(const double maxVoltage, const double updateRate);

        void update(const double escCommand, const double dt);

        // Getters
        double getVoltageOutput() const { return voltageOutput; }

    private:
        const double maxVoltage;
        const double updateRate;

        double voltageOutput{0.0};
        double timeSinceLastUpdate{0.0};
};