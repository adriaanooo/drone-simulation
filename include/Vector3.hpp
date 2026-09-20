#pragma once

struct Vector3
{
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Vector3 operator+(const Vector3& other) const
    {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vector3 operator-(const Vector3& other) const
    {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vector3 operator*(double scalar) const
    {
        return {x * scalar, y * scalar, z * scalar};
    }

    Vector3 crossProduct(const Vector3& other) const
    {
        return {
            y * other.z - z * other.y,
            x * other.z - z * other.x,
            x * other.y - y * other.x
        };
    }
};