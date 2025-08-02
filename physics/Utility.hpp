#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

typedef struct{

    float x, y;
    
    Vec2(float x = 0, float y = 0) : x(x), y(y) {}
    
    Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }
    
    Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }
    
    Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }
    
    Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
    
    float length() const {
        return std::sqrt(x*x + y*y);
    }
    
    Vec2 normalized() const {
        float len = length();
        if (len > 0) {
            return Vec2(x/len, y/len);
        }
        return *this;
    }
}Vec2;

float randFloat(float min, float max) {
    return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

#endif