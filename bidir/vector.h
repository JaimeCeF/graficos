#ifndef VECTOR_H
#define VECTOR_H

#include <cmath>

class Vector
{
public:
    double x, y, z; // coordinates x, y, z

    // Constructor for the vector, parameters default to zero
    Vector(double x_ = 0, double y_ = 0, double z_ = 0) { x = x_; y = y_; z = z_; }

    // Operator for vector addition and subtraction
    Vector operator+(const Vector& b) const { return Vector(x + b.x, y + b.y, z + b.z); }
    Vector operator-(const Vector& b) const { return Vector(x - b.x, y - b.y, z - b.z); }

    // Operator for vector-scalar multiplication
    Vector operator*(double b) const { return Vector(x * b, y * b, z * b); }

    // Operator % for cross product
    Vector operator%(const Vector& b) const { return Vector(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x); }

    // Dot product with vector b
    double dot(const Vector& b) const { return x * b.x + y * b.y + z * b.z; }

    // Element-wise product (Hadamard product)
    Vector mult(const Vector& b) const { return Vector(x * b.x, y * b.y, z * b.z); }

    // Normalize vector
    Vector& normalize() { return *this = *this * (1.0 / std::sqrt(x * x + y * y + z * z)); }

    // Calculate squared distance between this vector and another vector
    double squaredDistance(const Vector& b) const
    {
        double dx = x - b.x;
        double dy = y - b.y;
        double dz = z - b.z;
        return dx * dx + dy * dy + dz * dz;
    }

    friend std::ostream& operator<<(std::ostream& os, const Vector& vec)
    {
        os << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
        return os;
    }
};

typedef Vector Point;
typedef Vector Color;

#endif // VECTOR_H