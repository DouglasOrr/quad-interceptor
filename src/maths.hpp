#pragma once

#include <cmath>

using uint = unsigned int;
using uint64 = unsigned long long;

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

    float& operator[](uint row, uint col) { return data[col * M + row]; }
    const float& operator[](uint row, uint col) const { return data[col * M + row]; }
};

using Mat3 = Mat<3, 3>;
using Mat4 = Mat<4, 4>;

////////////////////////////////////////////////////////////////////////////////
// Operations (operators; factories; transforms)
//
// Note naming scheme: factories that build a Mat/Vec from raw inputs carry a
// `mat_`/`vec_` prefix; operations on existing values take no prefix.

template <uint N>
Vec<N> operator+(Vec<N> a, const Vec<N>& b) {
    for (uint i = 0; i < N; ++i)
        a[i] += b[i];
    return a;
}

template <uint N>
Vec<N>& operator+=(Vec<N>& a, const Vec<N>& b) {
    a = a + b;
    return a;
}

template <uint N>
Vec<N> operator*(float a, Vec<N> b) {
    for (uint i = 0; i < N; ++i)
        b[i] *= a;
    return b;
}

inline Quat operator+(const Quat& a, const Quat& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]};
}

inline Quat operator*(const Quat& a, const Quat& b) {
    float ax = a[0], ay = a[1], az = a[2], aw = a[3];
    float bx = b[0], by = b[1], bz = b[2], bw = b[3];
    return {
        aw * bx + bw * ax + ay * bz - az * by,
        aw * by + bw * ay + az * bx - ax * bz,
        aw * bz + bw * az + ax * by - ay * bx,
        aw * bw - ax * bx - ay * by - az * bz,
    };
}

inline Quat operator*(float s, const Quat& q) {
    return {s * q[0], s * q[1], s * q[2], s * q[3]};
}

inline Quat quat_pure(Vec<3> v) {
    return {v[0], v[1], v[2], 0.0f};
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

// Rigid transform [R | t] from a rotation and translation
inline Mat4 mat_rigid(Mat3 r, Vec<3> t) {
    Mat4 m{};
    for (uint i = 0; i < 3; ++i) {
        for (uint j = 0; j < 3; ++j)
            m[i, j] = r[i, j];
        m[i, 3] = t[i];
    }
    m[3, 3] = 1.0f;
    return m;
}

inline Mat3 mat_from(Quat q) {
    float x = q[0], y = q[1], z = q[2], w = q[3];
    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;
    Mat3 m{};
    m[0, 0] = 1.0f - 2.0f * (yy + zz);
    m[1, 0] = 2.0f * (xy + wz);
    m[2, 0] = 2.0f * (xz - wy);
    m[0, 1] = 2.0f * (xy - wz);
    m[1, 1] = 1.0f - 2.0f * (xx + zz);
    m[2, 1] = 2.0f * (yz + wx);
    m[0, 2] = 2.0f * (xz + wy);
    m[1, 2] = 2.0f * (yz - wx);
    m[2, 2] = 1.0f - 2.0f * (xx + yy);
    return m;
}

inline Mat4 mat_from(Quat q, Vec<3> t) {
    return mat_rigid(mat_from(q), t);
}

inline Quat normalise(Quat q) {
    float len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    float inv = len > 0.0f ? 1.0f / len : 0.0f;
    return {q[0] * inv, q[1] * inv, q[2] * inv, q[3] * inv};
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

template <uint M, uint N>
Mat<N, M> transpose(const Mat<M, N>& m) {
    Mat<N, M> r{};
    for (uint i = 0; i < M; ++i)
        for (uint j = 0; j < N; ++j)
            r[j, i] = m[i, j];
    return r;
}

template <uint M, uint N>
Vec<M> mul(const Mat<M, N>& a, const Vec<N>& v) {
    Vec<M> r{};
    for (uint row = 0; row < M; ++row) {
        float s = 0.0f;
        for (uint k = 0; k < N; ++k)
            s += a[row, k] * v[k];
        r[row] = s;
    }
    return r;
}

template <uint M, uint N>
Vec<N> mul(const Vec<M>& v, const Mat<M, N>& a) {
    Vec<N> r{};
    for (uint c = 0; c < N; ++c) {
        float s = 0.0f;
        for (uint k = 0; k < M; ++k)
            s += v[k] * a[k, c];
        r[c] = s;
    }
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
