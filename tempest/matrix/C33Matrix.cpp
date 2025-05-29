#include "tempest/matrix/C33Matrix.hpp"
#include "tempest/math/CMath.hpp"
#include "tempest/quaternion/C4Quaternion.hpp"

#include <storm/Error.hpp>


float C33Matrix::Det(float a, float b, float c, float d) {
    return (a * d) - (b * c);
}

C33Matrix C33Matrix::Rotation(float angle) {
    float cosa = cos(angle);
    float sina = sin(angle);

    C33Matrix result;
    result.a0 = cosa;
    result.a1 = sina;
    result.a2 = 0.0;

    result.b0 = -sina;
    result.b1 = cosa;
    result.b2 = 0.0;

    result.c0 = 0.0;
    result.c1 = 0.0;
    result.c2 = 1.0;
    return result;
}

C33Matrix C33Matrix::Rotation(float angle, const C3Vector& axis, bool unit) {
    C3Vector axis_ = axis;
    if (!unit) {
        axis_.Normalize();
    }
    STORM_ASSERT(axis_.IsUnit());

    float sina = CMath::sin(angle);
    float cosa = CMath::cos(angle);

    float xs = axis_.x * sina;
    float ys = axis_.y * sina;
    float zs = axis_.z * sina;

    float one_c = 1.0f - cosa;

    // https://en.wikipedia.org/wiki/Rotation_matrix
    // Rotation matrix from axis and angle
    C33Matrix result;

    result.a0 = axis_.x * axis_.x * one_c + cosa;
    result.a1 = axis_.x * axis_.y * one_c + zs;
    result.a2 = axis_.x * axis_.z * one_c - ys;

    result.b0 = axis_.x * axis_.y * one_c - zs;
    result.b1 = axis_.y * axis_.y * one_c + cosa;
    result.b2 = axis_.y * axis_.z * one_c + xs;

    result.c0 = axis_.x * axis_.z * one_c + ys;
    result.c1 = axis_.y * axis_.z * one_c - xs;
    result.c2 = axis_.z * axis_.z * one_c + cosa;

    return result;
}

C33Matrix& C33Matrix::operator+=(const C33Matrix& a) {
    this->a0 += a.a0;
    this->a1 += a.a1;
    this->a2 += a.a2;

    this->b0 += a.b0;
    this->b1 += a.b1;
    this->b2 += a.b2;

    this->c0 += a.c0;
    this->c1 += a.c1;
    this->c2 += a.c2;

    return *this;
}

C33Matrix& C33Matrix::operator-=(const C33Matrix& a) {
    this->a0 -= a.a0;
    this->a1 -= a.a1;
    this->a2 -= a.a2;

    this->b0 -= a.b0;
    this->b1 -= a.b1;
    this->b2 -= a.b2;

    this->c0 -= a.c0;
    this->c1 -= a.c1;
    this->c2 -= a.c2;

    return *this;
}

C33Matrix& C33Matrix::operator*=(float a) {
    this->a0 *= a;
    this->a1 *= a;
    this->a2 *= a;

    this->b0 *= a;
    this->b1 *= a;
    this->b2 *= a;

    this->c0 *= a;
    this->c1 *= a;
    this->c2 *= a;

    return *this;
}

C33Matrix& C33Matrix::operator*=(const C33Matrix& a) {
    *this = *this * a;
    return *this;
}

C33Matrix& C33Matrix::operator/=(float a) {
    this->a0 /= a;
    this->a1 /= a;
    this->a2 /= a;

    this->b0 /= a;
    this->b1 /= a;
    this->b2 /= a;

    this->c0 /= a;
    this->c1 /= a;
    this->c2 /= a;

    return *this;
}

void C33Matrix::Zero() {
    this->a0 = 0.0f;
    this->a1 = 0.0f;
    this->a2 = 0.0f;

    this->b0 = 0.0f;
    this->b1 = 0.0f;
    this->b2 = 0.0f;

    this->c0 = 0.0f;
    this->c1 = 0.0f;
    this->c2 = 0.0f;
}

void C33Matrix::Identity() {
    this->a0 = 1.0f;
    this->a1 = 0.0f;
    this->a2 = 0.0f;

    this->b0 = 0.0f;
    this->b1 = 1.0f;
    this->b2 = 0.0f;

    this->c0 = 0.0f;
    this->c1 = 0.0f;
    this->c2 = 1.0f;
}

float C33Matrix::Trace() {
    return this->a0 + this->b1 + this->c2;
}

void C33Matrix::Scale(float scale) {
    Scale(scale, scale, scale);
}

void C33Matrix::Scale(float x, float y) {
    this->a0 *= x;
    this->a1 *= x;

    this->b0 *= y;
    this->b1 *= y;
}

void C33Matrix::Scale(const C2Vector& scale) {
    Scale(scale.x, scale.y);
}

void C33Matrix::Scale(float x, float y, float z) {
    this->a0 *= x;
    this->a1 *= x;
    this->a2 *= x;

    this->b0 *= y;
    this->b1 *= y;
    this->b2 *= y;

    this->c0 *= z;
    this->c1 *= z;
    this->c2 *= z;
}

void C33Matrix::Scale(const C3Vector& scale) {
    Scale(scale.x, scale.y, scale.z);
}

void C33Matrix::Rotate(float angle) {
    *this = C33Matrix::Rotation(angle);
}

void C33Matrix::Rotate(const C4Quaternion& rotation) {
    float twox = 2.0f * rotation.x;
    float twoy = 2.0f * rotation.y;
    float twoz = 2.0f * rotation.z;

    C33Matrix left;

    left.a0 = 1.0f - ((twoy * rotation.y) + (twoz * rotation.z));
    left.a1 = (twoy * rotation.x) + (twoz * rotation.w);
    left.a2 = (twoz * rotation.x) - (twoy * rotation.w);

    left.b0 = (twoy * rotation.x) - (twoz * rotation.w);
    left.b1 = 1.0f - ((twox * rotation.x) + (twoz * rotation.z));
    left.b2 = (twoz * rotation.y) + (twox * rotation.w);

    left.c0 = (twoz * rotation.x) + (twoy * rotation.w);
    left.c1 = (twoz * rotation.y) - (twox * rotation.w);
    left.c2 = 1.0f - ((twox * rotation.x) + (twoy * rotation.y));

    *this = left * (*this);
}

void C33Matrix::Rotate(float angle, const C3Vector& axis, bool unit) {
    *this = C33Matrix::Rotation(angle, axis, unit) * (*this);
}

void C33Matrix::Translate(const C2Vector& move) {
    this->c0 += this->a0 * move.x + this->b0 * move.y;
    this->c1 += this->a1 * move.x + this->b1 * move.y;
}

C33Matrix C33Matrix::Transpose() const {
    return {
        this->a0,
        this->b0,
        this->c0,
        this->a1,
        this->b1,
        this->c1,
        this->a2,
        this->b2,
        this->c2
    };
}

float C33Matrix::Determinant() const {
 return this->c0 * this->a1 * this->b2
    + this->a2 * this->b0 * this->c1
    + this->c2 * this->a0 * this->b1
    - this->c0 * this->a2 * this->b1
    - this->c2 * this->a1 * this->b0
    - this->a0 * this->c1 * this->b2;
}

C33Matrix C33Matrix::Cofactors() const {
    float a0 = C33Matrix::Det(this->b1, this->b2, this->c1, this->c2);
    float a1 = -C33Matrix::Det(this->b0, this->b2, this->c0, this->c2);
    float a2 = C33Matrix::Det(this->b0, this->b1, this->c0, this->c1);

    float b0 = -C33Matrix::Det(this->a1, this->a2, this->c1, this->c2);
    float b1 = C33Matrix::Det(this->a0, this->a2, this->c0, this->c2);
    float b2 = -C33Matrix::Det(this->a0, this->a1, this->c0, this->c1);

    float c0 = C33Matrix::Det(this->a1, this->a2, this->b1, this->b2);
    float c1 = -C33Matrix::Det(this->a0, this->a2, this->b0, this->b2);
    float c2 = C33Matrix::Det(this->a0, this->a1, this->b0, this->b1);

    return { a0, a1, a2, b0, b1, b2, c0, c1, c2 };
}

C33Matrix C33Matrix::Adjoint() const {
    float a0 = C33Matrix::Det(this->b1, this->b2, this->c1, this->c2);
    float a1 = -C33Matrix::Det(this->a1, this->a2, this->c1, this->c2);
    float a2 = C33Matrix::Det(this->a1, this->a2, this->b1, this->b2);

    float b0 = -C33Matrix::Det(this->b0, this->b2, this->c0, this->c2);
    float b1 = C33Matrix::Det(this->a0, this->a2, this->c0, this->c2);
    float b2 = -C33Matrix::Det(this->a0, this->a2, this->b0, this->b2);

    float c0 = C33Matrix::Det(this->b0, this->b1, this->c0, this->c1);
    float c1 = -C33Matrix::Det(this->a0, this->a1, this->c0, this->c1);
    float c2 = C33Matrix::Det(this->a0, this->a1, this->b0, this->b1);

    return { a0, a1, a2, b0, b1, b2, c0, c1, c2 };
}

C33Matrix C33Matrix::Inverse() const {
    return this->Inverse(this->Determinant());
}

C33Matrix C33Matrix::Inverse(float det) const {
    STORM_ASSERT(det != 0.0f);
    return this->Adjoint() * (1.0f / det);
}

C33Matrix C33Matrix::AffineInverse(const C3Vector& v) const {
    C33Matrix rotationScale = *this;
    C3Vector s = { 1.0f / v.x, 1.0f / v.y, 1.0f / v.z };
    rotationScale.Scale(s);
    C33Matrix matrix = rotationScale.Transpose();
    matrix.Scale(s);
    return matrix;
}

C33Matrix C33Matrix::AffineInverse(float a) const {
    if (CMath::fequalz(a, 1.0f, 0.00000095367432f)) {
        return this->Transpose();
    } else {
        C33Matrix matrix = this->Transpose();
        matrix.Scale(1.0f / (a * a));
        return matrix;
    }
}

C33Matrix operator*(const C33Matrix& l, float a) {
    float a0 = l.a0 * a;
    float a1 = l.a1 * a;
    float a2 = l.a2 * a;

    float b0 = l.b0 * a;
    float b1 = l.b1 * a;
    float b2 = l.b2 * a;

    float c0 = l.c0 * a;
    float c1 = l.c1 * a;
    float c2 = l.c2 * a;

    return { a0, a1, a2, b0, b1, b2, c0, c1, c2 };
}

C33Matrix operator*(const C33Matrix& l, const C33Matrix& r) {
    float a0 = l.a0 * r.a0 + l.a1 * r.b0 + l.a2 * r.c0;
    float a1 = l.a0 * r.a1 + l.a1 * r.b1 + l.a2 * r.c1;
    float a2 = l.a0 * r.a2 + l.a1 * r.b2 + l.a2 * r.c2;

    float b0 = l.b0 * r.a0 + l.b1 * r.b0 + l.b2 * r.c0;
    float b1 = l.b0 * r.a1 + l.b1 * r.b1 + l.b2 * r.c1;
    float b2 = l.b0 * r.a2 + l.b1 * r.b2 + l.b2 * r.c2;

    float c0 = l.c0 * r.a0 + l.c1 * r.b0 + l.c2 * r.c0;
    float c1 = l.c0 * r.a1 + l.c1 * r.b1 + l.c2 * r.c1;
    float c2 = l.c0 * r.a2 + l.c1 * r.b2 + l.c2 * r.c2;

    return { a0, a1, a2, b0, b1, b2, c0, c1, c2 };
}

C33Matrix operator/(const C33Matrix& l, float a) {
    float a0 = l.a0 / a;
    float a1 = l.a1 / a;
    float a2 = l.a2 / a;

    float b0 = l.b0 / a;
    float b1 = l.b1 / a;
    float b2 = l.b2 / a;

    float c0 = l.c0 / a;
    float c1 = l.c1 / a;
    float c2 = l.c2 / a;

    return { a0, a1, a2, b0, b1, b2, c0, c1, c2 };
}
