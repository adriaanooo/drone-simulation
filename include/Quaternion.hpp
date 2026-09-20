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

    void applyAngularRate(
        const Vector3& angularRate, 
        const double dt
    )
    {
        const Quaternion quaternionOld = {w, x, y, z};
        const Quaternion angularRateQuaternion = {
            0.0,
            angularRate.x,
            angularRate.y,
            angularRate.z
        };
        const Quaternion quaternionDelta = angularRateQuaternion * (0.5 * dt);
        Quaternion quaternionNew = quaternionOld + quaternionOld * quaternionDelta;
        quaternionNew.normalize();

        w = quaternionNew.w;
        x = quaternionNew.x;
        y = quaternionNew.y;
        z = quaternionNew.z;
    }

    Vector3 toEuler() const
    {
        const double sinr_cosp = 2.0 * (w * x + y * z);
        const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        const double roll = atan2(sinr_cosp, cosr_cosp);

        const double sinp = 2.0 * (w * y - z * x);
        const double pitch = asin(std::clamp(sinp, -1.0, 1.0));

        const double siny_cosp = 2.0 * (w * z + x * y);
        const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        const double yaw = atan2(siny_cosp, cosy_cosp);

        return {roll, pitch, yaw};
    }
};