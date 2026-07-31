#pragma once

#include <cmath>

using uint = unsigned int;

template <uint N>
struct Vec {
    float data[N];

    float& operator[](uint i) { return data[i]; }
    const float& operator[](uint i) const { return data[i]; }
};

struct Quat {
    float data[4];  // (x, y, z, w) where w is the scalar part

    float& operator[](uint i) { return data[i]; }
    const float& operator[](uint i) const { return data[i]; }
};

template <uint M, uint N>
struct Mat {
    float data[M * N];  // column-major

    float& operator[](uint row, uint col) { return data[col * N + row]; }
    const float& operator[](uint row, uint col) const { return data[col * N + row]; }
};

using Mat4 = Mat<4, 4>;

////////////////////////////////////////////////////////////////////////////////
// Operations
// Note naming scheme: factories that build a Mat/Vec from raw inputs carry a
// `mat_`/`vec_` prefix; operations on existing values take no prefix.

template <uint N>
Vec<N> add(Vec<N> a, const Vec<N>& b) {
    for (uint i = 0; i < N; ++i)
        a[i] += b[i];
    return a;
}

inline Mat4 mat_perspective(float fov_y, float aspect, float z_near, float z_far) {
    Mat4 m{};
    const float f = 1.0f / std::tan(fov_y / 2.0f);
    m[0, 0] = f / aspect;
    m[1, 1] = f;
    m[2, 2] = (z_far + z_near) / (z_near - z_far);
    m[3, 2] = -1.0f;
    m[2, 3] = (2.0f * z_far * z_near) / (z_near - z_far);
    return m;
}

inline Mat4 mat_scale(float x, float y, float z) {
    Mat4 m{};
    m[0, 0] = x;
    m[1, 1] = y;
    m[2, 2] = z;
    m[3, 3] = 1.0f;
    return m;
}

inline Mat4 mat_from(Quat q, Vec<3> t) {
    float x = q[0], y = q[1], z = q[2], w = q[3];
    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;
    Mat4 m{};
    m[0, 0] = 1.0f - 2.0f * (yy + zz);
    m[1, 0] = 2.0f * (xy + wz);
    m[2, 0] = 2.0f * (xz - wy);
    m[0, 1] = 2.0f * (xy - wz);
    m[1, 1] = 1.0f - 2.0f * (xx + zz);
    m[2, 1] = 2.0f * (yz + wx);
    m[0, 2] = 2.0f * (xz + wy);
    m[1, 2] = 2.0f * (yz - wx);
    m[2, 2] = 1.0f - 2.0f * (xx + yy);
    m[0, 3] = t[0];
    m[1, 3] = t[1];
    m[2, 3] = t[2];
    m[3, 3] = 1.0f;
    return m;
}

inline Vec<3> rotate(Quat q, Vec<3> v) {
    float qx = q[0], qy = q[1], qz = q[2], qw = q[3];
    float vx = v[0], vy = v[1], vz = v[2];

    // t = 2 * (q.xyz x v), then v' = v + qw*t + (q.xyz x t)
    float tx = 2.0f * (qy * vz - qz * vy);
    float ty = 2.0f * (qz * vx - qx * vz);
    float tz = 2.0f * (qx * vy - qy * vx);

    return {
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    };
}

// Inverse of a rigid [R|t] transform: [R^T | -R^T t].
inline Mat4 rigid_inverse(const Mat4& m) {
    Mat4 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r[i, j] = m[j, i];
    for (int i = 0; i < 3; ++i) {
        float s = 0.0f;
        for (int j = 0; j < 3; ++j)
            s += r[i, j] * m[j, 3];
        r[i, 3] = -s;
    }
    r[3, 3] = 1.0f;
    return r;
}

template <uint M, uint K, uint N>
Mat<M, N> mul(const Mat<M, K>& a, const Mat<K, N>& b) {
    Mat<M, N> r{};
    for (uint c = 0; c < N; ++c)
        for (uint row = 0; row < M; ++row) {
            float s = 0.0f;
            for (uint k = 0; k < K; ++k)
                s += a[row, k] * b[k, c];
            r[row, c] = s;
        }
    return r;
}
