#ifndef VEC2_HPP
#define VEC2_HPP

#include <cmath>

class Vec3{
    private:
        float x, y, z;

    public:
        Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
        Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

        Vec3 operator+(const Vec3& other) const { return Vec3(x + other.x, y + other.y, z + other.z); }
        Vec3 operator-(const Vec3& other) const { return Vec3(x - other.x, y - other.y, z - other.z); }
        Vec3 operator*(const float scalar) const { return Vec3(x * scalar, y * scalar, z * scalar); }
        Vec3 operator/(const float scalar) const { return Vec3(x / scalar, y / scalar, z / scalar); }

        Vec3& operator+=(const Vec3& other) { x += other.x; y += other.y; z += other.z; return *this; }
        Vec3& operator-=(const Vec3& other) { x -= other.x; y -= other.y; z -= other.z; return *this; }
        Vec3& operator*=(float scalar) { x *= scalar; y *= scalar; z *= scalar; return *this; }
        Vec3& operator/=(float scalar) { x /= scalar; y /= scalar; z /= scalar; return *this; }

        float getX() const { return x; }
        float getY() const { return y; }
        float getZ() const { return z; }
        void setX(float val) { x = val; }
        void setY(float val) { y = val; }
        void setZ(float val) { z = val; }
        float length() const { return std::sqrt(x * x + y * y); }

        void clear() { x = 0.0f; y = 0.0f; z = 0.0f; }
        static Vec3 zero() { return Vec3(0.0f, 0.0f, 0.0f); }
};

#endif