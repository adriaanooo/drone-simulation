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
        double update(double rollActual, double pitchActual, double yawActual);

    private:
        Vector3 gyroBias;

        double gyroNoiseStd;
        double walkStd;

        std::mt19937 gen;
        std::normal_distribution<double> dist;

        const Vector3 gravity{0.0, 0.0, -9.81};

        double nextGaussian() {
            return dist(gen);
        }
};