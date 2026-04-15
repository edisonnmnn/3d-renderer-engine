#pragma once
#include <cmath>
#include "vec3.h"

struct Mat4 {
    float m[4][4] = {};

    static Mat4 identity() {
        Mat4 r;
        for (int i = 0; i < 4; i++) r.m[i][i] = 1.0f;
        return r;
    }

    static Mat4 translate(float tx, float ty, float tz) {
        Mat4 r = identity();
        r.m[0][3] = tx; r.m[1][3] = ty; r.m[2][3] = tz;
        return r;
    }

    static Mat4 rotateX(float a) {
        Mat4 r = identity();
        r.m[1][1] =  std::cos(a); r.m[1][2] = -std::sin(a);
        r.m[2][1] =  std::sin(a); r.m[2][2] =  std::cos(a);
        return r;
    }

    static Mat4 rotateY(float a) {
        Mat4 r = identity();
        r.m[0][0] =  std::cos(a); r.m[0][2] =  std::sin(a);
        r.m[2][0] = -std::sin(a); r.m[2][2] =  std::cos(a);
        return r;
    }

    static Mat4 rotateZ(float a) {
        Mat4 r = identity();
        r.m[0][0] =  std::cos(a); r.m[0][1] = -std::sin(a);
        r.m[1][0] =  std::sin(a); r.m[1][1] =  std::cos(a);
        return r;
    }

    Mat4 operator*(const Mat4& b) const {
        Mat4 result;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                for (int p = 0; p < 4; p++)
                    result.m[i][j] += m[i][p] * b.m[p][j];
        return result;
    }

    // Treats Vec3 as a point (w=1), drops w from result
    Vec3 operator*(const Vec3& v) const {
        return Vec3(
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3],
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3],
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3]
        );
    }
};
