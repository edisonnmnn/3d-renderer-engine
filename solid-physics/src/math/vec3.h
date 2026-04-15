#pragma once
#include <cmath>

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& b) const { return Vec3(x+b.x, y+b.y, z+b.z); }
    Vec3 operator-(const Vec3& b) const { return Vec3(x-b.x, y-b.y, z-b.z); }
    Vec3 operator*(float n)       const { return Vec3(x*n,   y*n,   z*n);   }
    Vec3 operator-()              const { return Vec3(-x, -y, -z); }

    float dot(const Vec3& b)  const { return x*b.x + y*b.y + z*b.z; }
    float length()            const { return std::sqrt(dot(*this)); }
    Vec3  normalized()        const { float l = length(); return Vec3(x/l, y/l, z/l); }
    Vec3  cross(const Vec3& b) const {
        return Vec3(y*b.z - z*b.y,
                    z*b.x - x*b.z,
                    x*b.y - y*b.x);
    }
};
