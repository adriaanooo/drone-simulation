#pragma once

#include <random>
#include <chrono>

struct Vector3
{
    double x {0.0}, y {0.0}, z {0.0};
};

class IMU
{
    public:
        IMU(double gyroNoiseSigma, double gyroUpdateRate);

        Vector3 update(Vector3 angularRate, double dt);

        Vector3 getAngularRate() const { return angularRateMeasured; }

    private:
        Vector3 gyroError;

        double gyroNoiseSigma;
        double gyroUpdateRate;
        double gyroUpdatePeriod;

        std::mt19937 gen;
        std::normal_distribution<double> noise;

        Vector3 angularRateMeasured{0.0, 0.0, 0.0};
        Vector3 previousAngulareRateMeasured{0.0, 0.0, 0.0};

        double timeSinceLastUpdate{0.0};
};