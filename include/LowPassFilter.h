#pragma once

#include "Vector3.hpp"

class LowPassFilter
{
    public:
        LowPassFilter(double alpha);
        void update(Vector3 input);

        Vector3 getOutput() const { return output; }

    private:
        double alpha;
        Vector3 output{0.0};
        Vector3 previousOutput{0.0};
};