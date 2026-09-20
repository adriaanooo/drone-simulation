#pragma once
#include "Vector3.hpp"

struct Matrix3
{
    double m[3][3]{};

    Vector3 operator*(const Vector3& v) const
    {
        return {
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        };
    }

    double det() const
    {
        return
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
            m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
            m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    Matrix3 inverse() const
    {
        const double d = det();

        if (std::abs(d) < 1e-12)
        {
            return {};
        }

        Matrix3 result{};

        const double a = m[0][0], b = m[0][1], c = m[0][2];
        const double d0 = m[1][0], e = m[1][1], f = m[1][2];
        const double g = m[2][0], h = m[2][1], i = m[2][2];

        result.m[0][0] = (e * i - f * h) / d;
        result.m[0][1] = (c * h - b * i) / d;
        result.m[0][2] = (b * f - c * e) / d;

        result.m[1][0] = (f * g - d0 * i) / d;
        result.m[1][1] = (a * i - c * g) / d;
        result.m[1][2] = (c * d0 - a * f) / d;

        result.m[2][0] = (d0 * h - e * g) / d;
        result.m[2][1] = (b * g - a * h) / d;
        result.m[2][2] = (a * e - b * d0) / d;

        return result;
    }
};