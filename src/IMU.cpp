#include "IMU.hpp"
#include <random>
#include <chrono>

IMU::IMU(double gyroNoiseSigma, double gyroUpdateRate)
    : gyroNoiseSigma(gyroNoiseSigma),
    noise(0.0, gyroNoiseSigma),
    gyroUpdatePeriod(1 / gyroUpdateRate)
{
    gen.seed(std::chrono::steady_clock::now().time_since_epoch().count());
}

void IMU::update(Vector3 angularRate, double dt)
{
    timeSinceLastUpdate += dt;

    if (timeSinceLastUpdate >= gyroUpdatePeriod)
    {
        angularRateMeasured.x = angularRate.x + noise(gen);
        angularRateMeasured.y = angularRate.y + noise(gen);
        angularRateMeasured.z = angularRate.z + noise(gen);
        timeSinceLastUpdate = 0.0;
    }
}