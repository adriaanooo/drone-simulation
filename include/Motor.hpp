#pragma once

class Motor {

    public:
        Motor(double maxVoltage, double KV, double thrustCoefficient, double torqueCoefficient);

        double getThrust(double appliedVoltage);
        double getTorque(double appliedVoltage);
        double getRPM() const { return rpm; }

    private:
        double maxVoltage;
        double KV;
        double thrustCoefficient;
        double torqueCoefficient;
        double rpm{0.0};
};