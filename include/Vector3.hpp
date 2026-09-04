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
};