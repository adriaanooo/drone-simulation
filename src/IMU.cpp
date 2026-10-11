#include "IMU.hpp"

IMU::IMU(
    double gyroNoiseSigma, 
    double gyroUpdateRate
)
    : gyroNoiseSigma(gyroNoiseSigma),
    noise(0.0, gyroNoiseSigma),
    gyroUpdateRate(gyroUpdateRate)
{
    gen.seed(std::chrono::steady_clock::now().time_since_epoch().count());
}

void IMU::update(const Vector3 droneAngularRates, const double dt)
{
    timeSinceLastUpdate += dt;

    if (timeSinceLastUpdate >= (1 / gyroUpdateRate))
    {
        const Vector3 noiseVector = {
            noise(gen),
            noise(gen),
            noise(gen)
        };
        angularRates = droneAngularRates + noiseVector;
        timeSinceLastUpdate = 0.0;
    }
}