#pragma once
#include <vector>

struct Step
{
    double time;
    double rollRate;
    double pitchRate;
    double yawRate;
};

class StepController
{
    public:
        StepController(const std::vector<Step>& steps);

        void update(double time);

        double getRollRateTarget() const { return rollRateTarget; }
        double getPitchRateTarget() const { return pitchRateTarget; }
        double getYawRateTarget() const { return yawRateTarget; }

    private:
        std::vector<Step> steps;

        double rollRateTarget{0.0};
        double pitchRateTarget{0.0};
        double yawRateTarget{0.0};
};