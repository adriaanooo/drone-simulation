#pragma once

#include <random>
#include <chrono>
#include "Vector3.hpp"

class IMU
{
    public:
        IMU(const double gyroNoiseSigma, const double gyroUpdateRate);

        void update(const Vector3 droneAngularRates, const double dt);

        Vector3 getAngularRates() const { return angularRates; }

    private:
        Vector3 gyroError;

        const double gyroNoiseSigma;
        const double gyroUpdateRate;

        std::mt19937 gen;
        std::normal_distribution<double> noise;

        Vector3 angularRates{0.0, 0.0, 0.0};
        Vector3 previousAngulareRates{0.0, 0.0, 0.0};

        double timeSinceLastUpdate{0.0};
};