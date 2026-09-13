#pragma once

class ESC
{
    public:
        ESC(const double maxVoltage, const double updateRate);

        void update(double throttleInput, double dt);

        double getVoltageOutput() const { return voltageOutput; }
        double getMaxVoltage() const { return maxVoltage; }

    private:
        double maxVoltage;
        double updateRate;
        double updatePeriod;

        double throttleInput{0.0};
        double voltageOutput{0.0};
        double timeSinceLastUpdate{0.0};
};