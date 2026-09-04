#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(double alpha) 
    : alpha(alpha)
{
}

void LowPassFilter::update(Vector3 input)
{
    output = previousOutput + (input - previousOutput) * alpha;
    previousOutput = output;
}
