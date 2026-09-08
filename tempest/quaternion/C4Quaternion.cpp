#include "tempest/quaternion/C4Quaternion.hpp"

#include <cmath>
#include <cstdint>

// OFFSET: 0x4F4320
C4Quaternion operator*(const C4Quaternion& l, const C4Quaternion& r) {
    C4Quaternion result;

    result.x = (l.w * r.x) + (r.z * l.y) + (r.w * l.x) - (l.z * r.y);
    result.y = (r.w * l.y) + (l.w * r.y) + (l.z * r.x) - (r.z * l.x);
    result.z = (r.z * l.w) + (r.w * l.z) + (l.x * r.y) - (r.x * l.y);
    result.w = (r.w * l.w) - (l.x * r.x) - (l.y * r.y) - (r.z * l.z);

    return result;
}

C4Quaternion C4Quaternion::Nlerp(float ratio, const C4Quaternion& q1, const C4Quaternion& q2) {
    float x = (q2.x - q1.x) * ratio + q1.x;
    float y = (q2.y - q1.y) * ratio + q1.y;
    float z = (q2.z - q1.z) * ratio + q1.z;
    float w = (q2.w - q1.w) * ratio + q1.w;

    float m = x * x + y * y + z * z + w * w;
    float v9 = ((m - 0.95906597) * -0.532516) + 1.021435;

    if (m <= 0.91521198) {
        v9 *= (((v9 * v9 * m) - 0.95906597) * -0.532516) + 1.021435;

        if (m <= 0.6521197) {
            v9 *= (((v9 * v9 * m) - 0.95906597) * -0.532516) + 1.021435;
        }
    }

    x *= v9;
    y *= v9;
    z *= v9;
    w *= v9;

    return { x, y, z, w };
}

// OFFSET: 0x982460
C4Quaternion C4Quaternion::Slerp(float ratio, const C4Quaternion& q1, const C4Quaternion& q2) {
    float dot = q1.z * q2.z + q1.y * q2.y + q1.x * q2.x + q2.w * q1.w;
    float sign = 1.0f;

    if (dot < 0.0f) {
        sign = -1.0f;
        dot = -dot;
    }

    float sinAngle = sqrt(fabs(1.0f - dot * dot));

    if (fabs(sinAngle) < 0.00000047683716f) {
        return q1;
    }

    float angle = atan2(sinAngle, dot);
    float invSinAngle = 1.0f / sinAngle;
    float weight1 = sin((1.0f - ratio) * angle) * invSinAngle;
    float weight2 = invSinAngle * sin(ratio * angle) * sign;

    float x = q2.x * weight2 + q1.x * weight1;
    float y = weight2 * q2.y + q1.y * weight1;
    float z = q1.z * weight1 + weight2 * q2.z;
    float w = weight2 * q2.w + weight1 * q1.w;

    return { x, y, z, w };
}

// OFFSET: 0x979110
void C4Quaternion::Normalize() {
    float lengthSq = (this->w * this->w) + (this->z * this->z) + (this->y * this->y) + (this->x * this->x);

    if (lengthSq > 2.384185791015625e-07f) {
        float invLength = 1.0f / sqrt(lengthSq);

        this->x = this->x * invLength;
        this->y = this->y * invLength;
        this->z = invLength * this->z;
        this->w = invLength * this->w;
    }
}

// OFFSET: 0x9828B0
void C4Quaternion::FromBasis(const float* basis) {
    float transposed[9];

    transposed[0] = basis[0];
    transposed[1] = basis[3];
    transposed[2] = basis[6];
    transposed[3] = basis[1];
    transposed[4] = basis[4];
    transposed[5] = basis[7];
    transposed[6] = basis[2];
    transposed[7] = basis[5];
    transposed[8] = basis[8];

    this->FromRotationMatrix(basis[0] + basis[4] + basis[8], transposed);
}

// OFFSET: 0x9826A0
void C4Quaternion::FromRotationMatrix(float trace, const float* m) {
    static const int32_t next[3] = { 1, 2, 0 };

    float* q = &this->x;

    if (trace > 0.0f) {
        float s = sqrt(trace + 1.0f);
        float t = 0.5f / s;

        q[3] = s * 0.5f;
        q[0] = (m[7] - m[5]) * t;
        q[1] = (m[2] - m[6]) * t;
        q[2] = t * (m[3] - m[1]);
        return;
    }

    int32_t i = m[0] < m[4] ? 1 : 0;

    if (m[4 * i] < m[8]) {
        i = 2;
    }

    int32_t j = next[i];
    int32_t k = next[j];

    float s = sqrt(m[4 * i] - m[4 * j] - m[4 * k] + 1.0f);
    float t = 0.5f / s;

    q[i] = s * 0.5f;
    q[3] = (m[3 * k + j] - m[3 * j + k]) * t;
    q[j] = (m[3 * i + j] + m[3 * j + i]) * t;
    q[k] = t * (m[3 * i + k] + m[3 * k + i]);
}
