#pragma once

#include <cmath>
#include "Vector3.hpp"

struct Quaternion
{
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Quaternion operator*(const Quaternion& other) const
    {
        return {
            w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w
        };
    }

    Quaternion operator*(double scalar) const
    {
        return {
            w * scalar,
            x * scalar,
            y * scalar,
            z * scalar
        };
    }

    Quaternion operator+(const Quaternion& other) const
    {
        return {
            w + other.w,
            x + other.x,
            y + other.y,
            z + other.z
        };
    }

    double norm() const
    {
        return sqrt(w * w + x * x + y * y + z * z);
    }

    void normalize()
    {
        const double length = norm();

        if (length > 0.0)
        {
            w /= length;
            x /= length;
            y /= length;
            z /= length;
        }
    }

    Quaternion conjugate() const
    {
        return {w, -x, -y, -z};
    }

    static Quaternion fromAngularRate(
        const Vector3& angularRate, 
        const Quaternion& quaternionOld,
        const double dt)
    {
        const Quaternion angularRateScaled = {
            0.0,
            angularRate.x * dt,
            angularRate.y * dt,
            angularRate.z * dt
        };

        const Quaternion derivative = quaternionOld * angularRateScaled;

        return {
            quaternionOld + derivative * 0.5
        };
    }
};