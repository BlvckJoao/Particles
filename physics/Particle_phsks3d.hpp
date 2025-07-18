#ifndef PARTICLE_PHSX3D_HPP
#define PARTICLE_PHSX3D_HPP

#include <vector>
#include <cmath>

typedef struct{
    float x, y, z;

    Vec3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    Vec3 operator*(float scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    float length() const {
        return std::sqrt(x*x + y*y + z*z);
    }

    Vec3 normalized() const {
        float len = length();
        if (len > 0) {
            return Vec3(x/len, y/len, z/len);
        }
        return *this;
    }
}Vec3;

struct Particle3D {
    Vec3 position;
    Vec3 velocity;
    Vec3 acceleration;
    float mass;
    float radius;

    Particle3D(Vec3 pos = Vec3(), Vec3 vel = Vec3(), Vec3 acc = Vec3(), float m = 1.0f, float r = 1.0f)
        : position(pos), velocity(vel), acceleration(acc), mass(m), radius(r) {}
};

#endif