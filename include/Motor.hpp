#pragma once

class Motor {

    public:
        Motor(double maxVoltage, double KV, double k);

        double getThrust(double appliedVoltage);
        double getRPM() const { return rpm; }

    private:
        double maxVoltage;
        double KV;
        double k;
        double rpm{0.0};
};