#include "Controller.hpp"

StepController::StepController(const std::vector<Step>& steps)
    : steps(steps)
{
}

void StepController::update(double time)
{
    for (const Step& step : steps)
    {
        if (time >= step.time)
        {
            rollRateTarget = step.rollRate;
            pitchRateTarget = step.pitchRate;
            yawRateTarget = step.yawRate;
        }
        else
        {
            break;
        }
    }
}