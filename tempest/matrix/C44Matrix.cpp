#include "tempest/matrix/C44Matrix.hpp"
#include "tempest/math/CMath.hpp"
#include "tempest/matrix/C33Matrix.hpp"
#include "tempest/quaternion/C4Quaternion.hpp"
#include "tempest/vector/C3Vector.hpp"
#include "tempest/box/CAaBox.hpp"
#include <storm/Error.hpp>

float C44Matrix::Det(float a, float b, float c, float d, float e, float f, float g, float h, float i) {
    return (b * f * g) + (c * d * h) + (a * e * i) - (c * e * g) - (b * d * i) - (a * f * h);
}

C44Matrix C44Matrix::Rotation(float angle, const C3Vector& axis, bool unit) {
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

    C44Matrix result;

    result.a0 = axis_.x * axis_.x * one_c + cosa;
    result.a1 = axis_.x * axis_.y * one_c + zs;
    result.a2 = axis_.x * axis_.z * one_c - ys;
    result.a3 = 0.0f;

    result.b0 = axis_.x * axis_.y * one_c - zs;
    result.b1 = axis_.y * axis_.y * one_c + cosa;
    result.b2 = axis_.y * axis_.z * one_c + xs;
    result.b3 = 0.0f;

    result.c0 = axis_.x * axis_.z * one_c + ys;
    result.c1 = axis_.y * axis_.z * one_c - xs;
    result.c2 = axis_.z * axis_.z * one_c + cosa;
    result.c3 = 0.0f;

    result.d0 = 0.0f;
    result.d1 = 0.0f;
    result.d2 = 0.0f;
    result.d3 = 1.0f;

    return result;
}

C44Matrix C44Matrix::RotationAroundZ(float angle) {
    float cosAngle = cos(angle);
    float sinAngle = sin(angle);

    float a0 = cosAngle;
    float a1 = sinAngle;
    float a2 = 0.0f;
    float a3 = 0.0f;

    float b0 = -sinAngle;
    float b1 = cosAngle;
    float b2 = 0.0f;
    float b3 = 0.0f;

    float c0 = 0.0f;
    float c1 = 0.0f;
    float c2 = 1.0f;
    float c3 = 0.0f;

    float d0 = 0.0f;
    float d1 = 0.0f;
    float d2 = 0.0f;
    float d3 = 1.0f;

    return { a0, a1, a2, a3, b0, b1, b2, b3, c0, c1, c2, c3, d0, d1, d2, d3 };
}

C44Matrix C44Matrix::RotationAroundY(float angle) {
    float cosa = CMath::cos(angle);
    float sina = CMath::sin(angle);

    C44Matrix result;

    result.a0 = cosa;
    result.a1 = 0.0f;
    result.a2 = -sina;
    result.a3 = 0.0f;

    result.b0 = 0.0f;
    result.b1 = 1.0f;
    result.b2 = 0.0f;
    result.b3 = 0.0f;

    result.c0 = sina;
    result.c1 = 0.0f;
    result.c2 = cosa;
    result.c3 = 0.0f;

    result.d0 = 0.0f;
    result.d1 = 0.0f;
    result.d2 = 0.0f;
    result.d3 = 1.0f;

    return result;
}

C44Matrix C44Matrix::RotationAroundX(float angle) {
    float cosa = CMath::cos(angle);
    float sina = CMath::sin(angle);

    C44Matrix result;

    result.a0 = 1.0f;
    result.a1 = 0.0f;
    result.a2 = 0.0f;
    result.a3 = 0.0f;

    result.b0 = 0.0f;
    result.b1 = cosa;
    result.b2 = sina;
    result.b3 = 0.0f;

    result.c0 = 0.0f;
    result.c1 = -sina;
    result.c2 = cosa;
    result.c3 = 0.0f;

    result.d0 = 0.0f;
    result.d1 = 0.0f;
    result.d2 = 0.0f;
    result.d3 = 1.0f;

    return result;
}

C44Matrix::C44Matrix(const C4Vector& r0, const C4Vector& r1, const C4Vector& r2, const C4Vector& r3)
    : a0(r0.x)
    , a1(r0.y)
    , a2(r0.z)
    , a3(r0.w)
    , b0(r1.x)
    , b1(r1.y)
    , b2(r1.z)
    , b3(r1.w)
    , c0(r2.x)
    , c1(r2.y)
    , c2(r2.z)
    , c3(r2.w)
    , d0(r3.x)
    , d1(r3.y)
    , d2(r3.z)
    , d3(r3.w) {
}

C44Matrix::C44Matrix(float a0, float a1, float a2, float a3, float b0, float b1, float b2, float b3, float c0, float c1, float c2, float c3, float d0, float d1, float d2, float d3)
    : a0(a0)
    , a1(a1)
    , a2(a2)
    , a3(a3)
    , b0(b0)
    , b1(b1)
    , b2(b2)
    , b3(b3)
    , c0(c0)
    , c1(c1)
    , c2(c2)
    , c3(c3)
    , d0(d0)
    , d1(d1)
    , d2(d2)
    , d3(d3) {
}

C44Matrix::C44Matrix(const C33Matrix& m)
    : a0(m.a0)
    , a1(m.a1)
    , a2(m.a2)
    , a3(0.0f)
    , b0(m.b0)
    , b1(m.b1)
    , b2(m.b2)
    , b3(0.0f)
    , c0(m.c0)
    , c1(m.c1)
    , c2(m.c2)
    , c3(0.0f)
    , d0(0.0f)
    , d1(0.0f)
    , d2(0.0f)
    , d3(1.0f) {
}

C44Matrix::C44Matrix(const C4Quaternion& rotation) {
    this->a3 = 0.0f;
    this->b3 = 0.0f;
    this->c3 = 0.0f;

    this->d0 = 0.0f;
    this->d1 = 0.0f;
    this->d2 = 0.0f;
    this->d3 = 1.0f;

    float twox = rotation.x * 2.0f;
    float twoy = rotation.y * 2.0f;
    float twoz = rotation.z * 2.0f;

    float twx = rotation.w * twox;
    float twy = rotation.w * twoy;
    float twz = rotation.w * twoz;

    float txx = rotation.x * twox;
    float txy = rotation.x * twoy;
    float txz = rotation.x * twoz;

    float tyy = rotation.y * twoy;
    float tyz = rotation.y * twoz;
    float tzz = rotation.z * twoz;

    this->a0 = 1.0f - (tzz + tyy);
    this->a1 = txy + twz;
    this->a2 = txz - twy;
    this->b0 = txy - twz;
    this->b1 = 1.0f - (tzz + txx);
    this->b2 = tyz + twx;
    this->c0 = twy + txz;
    this->c1 = tyz - twx;
    this->c2 = 1.0f - (txx + tyy);
}

C44Matrix::C44Matrix(float a)
    : a0(a)
    , a1(a)
    , a2(a)
    , a3(a)
    , b0(a)
    , b1(a)
    , b2(a)
    , b3(a)
    , c0(a)
    , c1(a)
    , c2(a)
    , c3(a)
    , d0(a)
    , d1(a)
    , d2(a)
    , d3(a) {
}

C44Matrix C44Matrix::Adjoint() const {
    float a0 = this->b2 * this->c3 * this->d1 + this->b3 * this->c1 * this->d2 + this->b1 * this->c2 * this->d3 - this->b3 * this->c2 * this->d1 - this->b2 * this->c1 * this->d3 - this->b1 * this->c3 * this->d2;
    float a1 = -(this->a2 * this->c3 * this->d1 + this->a3 * this->c1 * this->d2 + this->a1 * this->c2 * this->d3 - this->a3 * this->c2 * this->d1 - this->a2 * this->c1 * this->d3 - this->a1 * this->c3 * this->d2);
    float a2 = this->a1 * this->b2 * this->d3 + this->a2 * this->b3 * this->d1 + this->a3 * this->b1 * this->d2 - this->a3 * this->b2 * this->d1 - this->a2 * this->b1 * this->d3 - this->a1 * this->b3 * this->d2;
    float a3 = -(this->a2 * this->b3 * this->c1 + this->a3 * this->b1 * this->c2 + this->a1 * this->b2 * this->c3 - this->a3 * this->b2 * this->c1 - this->a2 * this->b1 * this->c3 - this->a1 * this->b3 * this->c2);

    float b0 = -(this->b3 * this->c0 * this->d2 + this->b0 * this->c2 * this->d3 + this->b2 * this->c3 * this->d0 - this->b3 * this->c2 * this->d0 - this->b2 * this->c0 * this->d3 - this->b0 * this->c3 * this->d2);
    float b1 = this->a2 * this->c3 * this->d0 + this->a3 * this->c0 * this->d2 + this->a0 * this->c2 * this->d3 - this->a3 * this->c2 * this->d0 - this->a2 * this->c0 * this->d3 - this->a0 * this->c3 * this->d2;
    float b2 = -(this->a2 * this->b3 * this->d0 + this->a3 * this->b0 * this->d2 + this->a0 * this->b2 * this->d3 - this->a3 * this->b2 * this->d0 - this->a2 * this->b0 * this->d3 - this->a0 * this->b3 * this->d2);
    float b3 = this->a2 * this->b3 * this->c0 + this->a3 * this->b0 * this->c2 + this->a0 * this->b2 * this->c3 - this->a3 * this->b2 * this->c0 - this->a2 * this->b0 * this->c3 - this->a0 * this->b3 * this->c2;

    float c0 = this->b1 * this->c3 * this->d0 + this->b3 * this->c0 * this->d1 + this->b0 * this->c1 * this->d3 - this->b3 * this->c1 * this->d0 - this->b1 * this->c0 * this->d3 - this->b0 * this->c3 * this->d1;
    float c1 = -(this->a1 * this->c3 * this->d0 + this->a3 * this->c0 * this->d1 + this->a0 * this->c1 * this->d3 - this->a3 * this->c1 * this->d0 - this->a1 * this->c0 * this->d3 - this->a0 * this->c3 * this->d1);
    float c2 = this->a1 * this->b3 * this->d0 + this->a3 * this->b0 * this->d1 + this->a0 * this->b1 * this->d3 - this->a3 * this->b1 * this->d0 - this->a1 * this->b0 * this->d3 - this->a0 * this->b3 * this->d1;
    float c3 = -(this->a1 * this->b3 * this->c0 + this->a3 * this->b0 * this->c1 + this->a0 * this->b1 * this->c3 - this->a3 * this->b1 * this->c0 - this->a1 * this->b0 * this->c3 - this->a0 * this->b3 * this->c1);

    float d0 = -(this->b2 * this->c0 * this->d1 + this->b0 * this->c1 * this->d2 + this->b1 * this->c2 * this->d0 - this->b2 * this->c1 * this->d0 - this->b1 * this->c0 * this->d2 - this->b0 * this->c2 * this->d1);
    float d1 = this->a1 * this->c2 * this->d0 + this->a2 * this->c0 * this->d1 + this->a0 * this->c1 * this->d2 - this->a2 * this->c1 * this->d0 - this->a1 * this->c0 * this->d2 - this->a0 * this->c2 * this->d1;
    float d2 = -(this->a1 * this->b2 * this->d0 + this->a2 * this->b0 * this->d1 + this->a0 * this->b1 * this->d2 - this->a2 * this->b1 * this->d0 - this->a1 * this->b0 * this->d2 - this->a0 * this->b2 * this->d1);
    float d3 = this->a0 * this->b1 * this->c2 + this->a1 * this->b2 * this->c0 + this->a2 * this->b0 * this->c1 - this->b1 * this->a2 * this->c0 - this->c2 * this->b0 * this->a1 - this->a0 * this->b2 * this->c1;

    return { a0, a1, a2, a3, b0, b1, b2, b3, c0, c1, c2, c3, d0, d1, d2, d3 };
}

/**
 * Calculate and return the inverse of this affine transform matrix. This function assumes no
 * scaling factor is present in the transform.
 *
 * @return Inverse of matrix
 */
// OFFSET: 0x4C2FC0
C44Matrix C44Matrix::AffineInverse() const {
    auto matrix = C44Matrix(C33Matrix(*this).Transpose());
    matrix.Translate(C3Vector(-this->d0, -this->d1, -this->d2));

    return matrix;
}

C44Matrix C44Matrix::AffineInverse(const C3Vector& v) const {
    C33Matrix rotationScale(*this);
    C3Vector s = { 1.0f / v.x, 1.0f / v.y, 1.0f / v.z };
    rotationScale.Scale(s);
    C44Matrix matrix(rotationScale.Transpose());
    matrix.Scale(s);
    C3Vector move(-this->d0, -this->d1, -this->d2);
    matrix.Translate(move);
    return matrix;
}

/**
 * Calculate and return the inverse of this affine transform matrix. This function assumes a
 * uniform scaling factor may be present in the transform.
 *
 * @param uniformScale Uniform scaling factor
 * @return Inverse of matrix
 */
C44Matrix C44Matrix::AffineInverse(float uniformScale) const {
    // No scaling factor present
    if (CMath::fequal(uniformScale, 1.0f)) {
        return this->AffineInverse();
    }

    // Uniform scaling factor present
    auto matrix = C44Matrix(C33Matrix(*this).Transpose());
    matrix.Scale(1.0f / (uniformScale * uniformScale));
    matrix.Translate(C3Vector(-this->d0, -this->d1, -this->d2));

    return matrix;
}

C44Matrix C44Matrix::Cofactors() const {
    float d3 = C44Matrix::Det(
        this->a0,
        this->a1,
        this->a2,
        this->b0,
        this->b1,
        this->b2,
        this->c0,
        this->c1,
        this->c2);
    float d2 = -C44Matrix::Det(
        this->a0,
        this->a1,
        this->a3,
        this->b0,
        this->b1,
        this->b3,
        this->c0,
        this->c1,
        this->c3);
    float d1 = C44Matrix::Det(
        this->a0,
        this->a2,
        this->a3,
        this->b0,
        this->b2,
        this->b3,
        this->c0,
        this->c2,
        this->c3);
    float d0 = -C44Matrix::Det(
        this->a1,
        this->a2,
        this->a3,
        this->b1,
        this->b2,
        this->b3,
        this->c1,
        this->c2,
        this->c3);
    float c3 = -C44Matrix::Det(
        this->a0,
        this->a1,
        this->a2,
        this->b0,
        this->b1,
        this->b2,
        this->d0,
        this->d1,
        this->d2);
    float c2 = C44Matrix::Det(
        this->a0,
        this->a1,
        this->a3,
        this->b0,
        this->b1,
        this->b3,
        this->d0,
        this->d1,
        this->d3);
    float c1 = -C44Matrix::Det(
        this->a0,
        this->a2,
        this->a3,
        this->b0,
        this->b2,
        this->b3,
        this->d0,
        this->d2,
        this->d3);
    float c0 = C44Matrix::Det(
        this->a1,
        this->a2,
        this->a3,
        this->b1,
        this->b2,
        this->b3,
        this->d1,
        this->d2,
        this->d3);
    float b3 = C44Matrix::Det(
        this->a0,
        this->a1,
        this->a2,
        this->c0,
        this->c1,
        this->c2,
        this->d0,
        this->d1,
        this->d2);
    float b2 = -C44Matrix::Det(
        this->a0,
        this->a1,
        this->a3,
        this->c0,
        this->c1,
        this->c3,
        this->d0,
        this->d1,
        this->d3);
    float b1 = C44Matrix::Det(
        this->a0,
        this->a2,
        this->a3,
        this->c0,
        this->c2,
        this->c3,
        this->d0,
        this->d2,
        this->d3);
    float b0 = -C44Matrix::Det(
        this->a1,
        this->a2,
        this->a3,
        this->c1,
        this->c2,
        this->c3,
        this->d1,
        this->d2,
        this->d3);
    float a3 = -C44Matrix::Det(
        this->b0,
        this->b1,
        this->b2,
        this->c0,
        this->c1,
        this->c2,
        this->d0,
        this->d1,
        this->d2);
    float a2 = C44Matrix::Det(
        this->b0,
        this->b1,
        this->b3,
        this->c0,
        this->c1,
        this->c3,
        this->d0,
        this->d1,
        this->d3);
    float a1 = -C44Matrix::Det(
        this->b0,
        this->b2,
        this->b3,
        this->c0,
        this->c2,
        this->c3,
        this->d0,
        this->d2,
        this->d3);
    float a0 = C44Matrix::Det(
        this->b1,
        this->b2,
        this->b3,
        this->c1,
        this->c2,
        this->c3,
        this->d1,
        this->d2,
        this->d3);
    return { a0, a1, a2, a3, b0, b1, b2, b3, c0, c1, c2, c3, d0, d1, d2, d3 };
}

C4Vector C44Matrix::Col0() const {
    return { this->a0, this->b0, this->c0, this->d0 };
}

C4Vector C44Matrix::Col1() const {
    return { this->a1, this->b1, this->c1, this->d1 };
}

C4Vector C44Matrix::Col2() const {
    return { this->a2, this->b2, this->c2, this->d2 };
}

C4Vector C44Matrix::Col3() const {
    return { this->a3, this->b3, this->c3, this->d3 };
}

float C44Matrix::Determinant() const {
    return (this->b1 * this->c2 * this->d3 + this->c3 * this->b2 * this->d1 + this->b3 * this->c1 * this->d2 - this->c2 * this->b3 * this->d1 - this->d3 * (this->c1 * this->b2) - this->b1 * this->c3 * this->d2) * this->a0 - (this->c2 * this->b0 * this->d3 + this->c3 * this->b2 * this->d0 + this->b3 * this->c0 * this->d2 - this->b3 * this->c2 * this->d0 - this->d3 * (this->c0 * this->b2) - this->b0 * this->c3 * this->d2) * this->a1 + (this->c1 * this->b0 * this->d3 + this->c3 * this->b1 * this->d0 + this->b3 * this->c0 * this->d1 - this->b3 * this->c1 * this->d0 - this->d3 * (this->c0 * this->b1) - this->b0 * this->c3 * this->d1) * this->a2 - (this->c1 * this->b0 * this->d2 + this->c2 * this->b1 * this->d0 + this->b2 * this->c0 * this->d1 - this->b2 * this->c1 * this->d0 - this->d2 * (this->c0 * this->b1) - this->b0 * this->c2 * this->d1) * this->a3;
}

void C44Matrix::Identity() {
    this->a0 = 1.0f;
    this->a1 = 0.0f;
    this->a2 = 0.0f;
    this->a3 = 0.0f;

    this->b0 = 0.0f;
    this->b1 = 1.0f;
    this->b2 = 0.0f;
    this->b3 = 0.0f;

    this->c0 = 0.0f;
    this->c1 = 0.0f;
    this->c2 = 1.0f;
    this->c3 = 0.0f;

    this->d0 = 0.0f;
    this->d1 = 0.0f;
    this->d2 = 0.0f;
    this->d3 = 1.0f;
}

C44Matrix C44Matrix::Inverse() const {
    return this->Inverse(this->Determinant());
}

C44Matrix C44Matrix::Inverse(float det) const {
    STORM_ASSERT(CMath::fequal(det, 0.0f) == false);
    return this->Adjoint() * (1.0f / det);
}

void C44Matrix::Rotate(const C4Quaternion& rotation) {
    *this = C44Matrix(rotation) * (*this);
}

void C44Matrix::Rotate(float angle, const C3Vector& axis, bool unit) {
    *this = C44Matrix::Rotation(angle, axis, unit) * (*this);
}

void C44Matrix::RotateAroundZ(float angle) {
    *this = C44Matrix::RotationAroundZ(angle) * (*this);
}

void C44Matrix::RotateAroundY(float angle) {
    *this = C44Matrix::RotationAroundY(angle) * (*this);
}

void C44Matrix::RotateAroundX(float angle) {
    *this = C44Matrix::RotationAroundX(angle) * (*this);
}

C4Vector C44Matrix::Row0() const {
    return { this->a0, this->a1, this->a2, this->a3 };
}

const C3Vector* C44Matrix::Row0AsVec3() const {
    return reinterpret_cast<const C3Vector*>(&this->a0);
}

C4Vector C44Matrix::Row1() const {
    return { this->b0, this->b1, this->b2, this->b3 };
}

const C3Vector* C44Matrix::Row1AsVec3() const {
    return reinterpret_cast<const C3Vector*>(&this->b0);
}

C4Vector C44Matrix::Row2() const {
    return { this->c0, this->c1, this->c2, this->c3 };
}

const C3Vector* C44Matrix::Row2AsVec3() const {
    return reinterpret_cast<const C3Vector*>(&this->c0);
}

C4Vector C44Matrix::Row3() const {
    return { this->d0, this->d1, this->d2, this->d3 };
}

const C3Vector* C44Matrix::Row3AsVec3() const {
    return reinterpret_cast<const C3Vector*>(&this->d0);
}

void C44Matrix::Scale(const C3Vector& scale) {
    this->a0 *= scale.x;
    this->a1 *= scale.x;
    this->a2 *= scale.x;

    this->b0 *= scale.y;
    this->b1 *= scale.y;
    this->b2 *= scale.y;

    this->c0 *= scale.z;
    this->c1 *= scale.z;
    this->c2 *= scale.z;
}

void C44Matrix::Scale(float scale) {
    this->a0 *= scale;
    this->a1 *= scale;
    this->a2 *= scale;

    this->b0 *= scale;
    this->b1 *= scale;
    this->b2 *= scale;

    this->c0 *= scale;
    this->c1 *= scale;
    this->c2 *= scale;
}

float C44Matrix::Trace() {
    return this->a0 + this->b1 + this->c2 + this->d3;
}

void C44Matrix::Translate(const C3Vector& move) {
    this->d0 = this->a0 * move.x + this->b0 * move.y + this->c0 * move.z + this->d0;
    this->d1 = this->a1 * move.x + this->b1 * move.y + this->c1 * move.z + this->d1;
    this->d2 = this->a2 * move.x + this->b2 * move.y + this->c2 * move.z + this->d2;
}

// OFFSET: 0x4C21B0
C3Vector C44Matrix::TransformPoint(const C3Vector& point) {
    C3Vector vec;
    vec.x = this->a0 * point.x + this->b0 * point.y + this->c0 * point.z + this->d0;
    vec.y = this->a1 * point.x + this->b1 * point.y + this->c1 * point.z + this->d1;
    vec.z = this->a2 * point.x + this->b2 * point.y + this->c2 * point.z + this->d2;
    return vec;
}

// OFFSET: 0x4C2270
C4Vector C44Matrix::TransformPoint(const C4Vector& point) {
    C4Vector vec;
    vec.x = this->d0 * point.w + this->b0 * point.y + this->c0 * point.z + this->a0 * point.x;
    vec.y = this->d1 * point.w + this->b1 * point.y + this->a1 * point.x + this->c1 * point.z;
    vec.z = this->d2 * point.w + this->b2 * point.y + this->a2 * point.x + this->c2 * point.z;
    vec.w = this->d3 * point.w + this->b3 * point.y + this->a3 * point.x + this->c3 * point.z;
    return vec;
}

// OFFSET: 0x57C2E0
C3Vector C44Matrix::TransformDirection(const C3Vector& v) {
    return {
        this->a0 * v.x + this->b0 * v.y + this->c0 * v.z,
        this->a1 * v.x + this->b1 * v.y + this->c1 * v.z,
        this->a2 * v.x + this->b2 * v.y + this->c2 * v.z,
    };
}

// OFFSET: 0x984860
CAaBox C44Matrix::Transform(CAaBox& box) {
    CAaBox out;
    out.b.x = this->d0;
    out.b.y = this->d1;
    out.b.z = this->d2;
    out.t = out.b;

    const float* basis = &this->a0;

    for (int i = 0; i < 3; ++i)
    {
        float* lo = &out.b.x + i;
        float* hi = &out.t.x + i;

        for (int j = 0; j < 3; ++j)
        {
            float k = basis[4 * j + i];
            float p = (&box.b.x)[j] * k;
            float q = (&box.t.x)[j] * k;

            if (q <= p) {
                *lo += q;
                *hi += p;
            } else {
                *lo += p;
                *hi += q;
            }
        }
    }

    return out;
}

C44Matrix C44Matrix::Transpose() const {
    float a0 = this->a0;
    float a1 = this->a1;
    float a2 = this->a2;
    float a3 = this->a3;

    float b0 = this->b0;
    float b1 = this->b1;
    float b2 = this->b2;
    float b3 = this->b3;

    float c0 = this->c0;
    float c1 = this->c1;
    float c2 = this->c2;
    float c3 = this->c3;

    float d0 = this->d0;
    float d1 = this->d1;
    float d2 = this->d2;
    float d3 = this->d3;

    return { a0, b0, c0, d0, a1, b1, c1, d1, a2, b2, c2, d2, a3, b3, c3, d3 };
}

void C44Matrix::Zero() {
    this->a0 = 0.0f;
    this->a1 = 0.0f;
    this->a2 = 0.0f;
    this->a3 = 0.0f;

    this->b0 = 0.0f;
    this->b1 = 0.0f;
    this->b2 = 0.0f;
    this->b3 = 0.0f;

    this->c0 = 0.0f;
    this->c1 = 0.0f;
    this->c2 = 0.0f;
    this->c3 = 0.0f;

    this->d0 = 0.0f;
    this->d1 = 0.0f;
    this->d2 = 0.0f;
    this->d3 = 0.0f;
}

C44Matrix& C44Matrix::operator+=(const C44Matrix& a) {
    this->a0 += a.a0;
    this->a1 += a.a1;
    this->a2 += a.a2;
    this->a3 += a.a3;

    this->b0 += a.b0;
    this->b1 += a.b1;
    this->b2 += a.b2;
    this->b3 += a.b3;

    this->c0 += a.c0;
    this->c1 += a.c1;
    this->c2 += a.c2;
    this->c3 += a.c3;

    this->d0 += a.d0;
    this->d1 += a.d1;
    this->d2 += a.d2;
    this->d3 += a.d3;

    return *this;
}

C44Matrix& C44Matrix::operator-=(const C44Matrix& a) {
    this->a0 -= a.a0;
    this->a1 -= a.a1;
    this->a2 -= a.a2;
    this->a3 -= a.a3;

    this->b0 -= a.b0;
    this->b1 -= a.b1;
    this->b2 -= a.b2;
    this->b3 -= a.b3;

    this->c0 -= a.c0;
    this->c1 -= a.c1;
    this->c2 -= a.c2;
    this->c3 -= a.c3;

    this->d0 -= a.d0;
    this->d1 -= a.d1;
    this->d2 -= a.d2;
    this->d3 -= a.d3;

    return *this;
}

C44Matrix& C44Matrix::operator*=(float a) {
    this->a0 *= a;
    this->a1 *= a;
    this->a2 *= a;
    this->a3 *= a;

    this->b0 *= a;
    this->b1 *= a;
    this->b2 *= a;
    this->b3 *= a;

    this->c0 *= a;
    this->c1 *= a;
    this->c2 *= a;
    this->c3 *= a;

    this->d0 *= a;
    this->d1 *= a;
    this->d2 *= a;
    this->d3 *= a;

    return *this;
}

C44Matrix& C44Matrix::operator*=(const C44Matrix& a) {
    *this = *this * a;
    return *this;
}

C44Matrix& C44Matrix::operator/=(float a) {
    this->a0 /= a;
    this->a1 /= a;
    this->a2 /= a;
    this->a3 /= a;

    this->b0 /= a;
    this->b1 /= a;
    this->b2 /= a;
    this->b3 /= a;

    this->c0 /= a;
    this->c1 /= a;
    this->c2 /= a;
    this->c3 /= a;

    this->d0 /= a;
    this->d1 /= a;
    this->d2 /= a;
    this->d3 /= a;

    return *this;
}

C44Matrix operator+(const C44Matrix& l, const C44Matrix& r) {
    return {
        l.a0 + r.a0,
        l.a1 + r.a1,
        l.a2 + r.a2,
        l.a3 + r.a3,

        l.b0 + r.b0,
        l.b1 + r.b1,
        l.b2 + r.b2,
        l.b3 + r.b3,

        l.c0 + r.c0,
        l.c1 + r.c1,
        l.c2 + r.c2,
        l.c3 + r.c3,

        l.d0 + r.d0,
        l.d1 + r.d1,
        l.d2 + r.d2,
        l.d3 + r.d3
    };
}

C44Matrix operator+(const C44Matrix& l, float a) {
    return {
        l.a0 + a,
        l.a1 + a,
        l.a2 + a,
        l.a3 + a,

        l.b0 + a,
        l.b1 + a,
        l.b2 + a,
        l.b3 + a,

        l.c0 + a,
        l.c1 + a,
        l.c2 + a,
        l.c3 + a,

        l.d0 + a,
        l.d1 + a,
        l.d2 + a,
        l.d3 + a,
    };
}

C44Matrix operator-(const C44Matrix& l, const C44Matrix& r) {
    return {
        l.a0 - r.a0,
        l.a1 - r.a1,
        l.a2 - r.a2,
        l.a3 - r.a3,

        l.b0 - r.b0,
        l.b1 - r.b1,
        l.b2 - r.b2,
        l.b3 - r.b3,

        l.c0 - r.c0,
        l.c1 - r.c1,
        l.c2 - r.c2,
        l.c3 - r.c3,

        l.d0 - r.d0,
        l.d1 - r.d1,
        l.d2 - r.d2,
        l.d3 - r.d3
    };
}

C44Matrix operator-(const C44Matrix& l, float a) {
    return {
        l.a0 - a,
        l.a1 - a,
        l.a2 - a,
        l.a3 - a,

        l.b0 - a,
        l.b1 - a,
        l.b2 - a,
        l.b3 - a,

        l.c0 - a,
        l.c1 - a,
        l.c2 - a,
        l.c3 - a,

        l.d0 - a,
        l.d1 - a,
        l.d2 - a,
        l.d3 - a,
    };
}

C44Matrix operator*(const C44Matrix& l, const C44Matrix& r) {
    float a0 = l.a0 * r.a0 + l.a1 * r.b0 + l.a2 * r.c0 + l.a3 * r.d0;
    float a1 = l.a0 * r.a1 + l.a1 * r.b1 + l.a2 * r.c1 + l.a3 * r.d1;
    float a2 = l.a0 * r.a2 + l.a1 * r.b2 + l.a2 * r.c2 + l.a3 * r.d2;
    float a3 = l.a0 * r.a3 + l.a1 * r.b3 + l.a2 * r.c3 + l.a3 * r.d3;

    float b0 = l.b0 * r.a0 + l.b1 * r.b0 + l.b2 * r.c0 + l.b3 * r.d0;
    float b1 = l.b0 * r.a1 + l.b1 * r.b1 + l.b2 * r.c1 + l.b3 * r.d1;
    float b2 = l.b0 * r.a2 + l.b1 * r.b2 + l.b2 * r.c2 + l.b3 * r.d2;
    float b3 = l.b0 * r.a3 + l.b1 * r.b3 + l.b2 * r.c3 + l.b3 * r.d3;

    float c0 = l.c0 * r.a0 + l.c1 * r.b0 + l.c2 * r.c0 + l.c3 * r.d0;
    float c1 = l.c0 * r.a1 + l.c1 * r.b1 + l.c2 * r.c1 + l.c3 * r.d1;
    float c2 = l.c0 * r.a2 + l.c1 * r.b2 + l.c2 * r.c2 + l.c3 * r.d2;
    float c3 = l.c0 * r.a3 + l.c1 * r.b3 + l.c2 * r.c3 + l.c3 * r.d3;

    float d0 = l.d0 * r.a0 + l.d1 * r.b0 + l.d2 * r.c0 + l.d3 * r.d0;
    float d1 = l.d0 * r.a1 + l.d1 * r.b1 + l.d2 * r.c1 + l.d3 * r.d1;
    float d2 = l.d0 * r.a2 + l.d1 * r.b2 + l.d2 * r.c2 + l.d3 * r.d2;
    float d3 = l.d0 * r.a3 + l.d1 * r.b3 + l.d2 * r.c3 + l.d3 * r.d3;

    return { a0, a1, a2, a3, b0, b1, b2, b3, c0, c1, c2, c3, d0, d1, d2, d3 };
}

C44Matrix operator*(const C44Matrix& l, float a) {
    return {
        l.a0 * a,
        l.a1 * a,
        l.a2 * a,
        l.a3 * a,

        l.b0 * a,
        l.b1 * a,
        l.b2 * a,
        l.b3 * a,

        l.c0 * a,
        l.c1 * a,
        l.c2 * a,
        l.c3 * a,

        l.d0 * a,
        l.d1 * a,
        l.d2 * a,
        l.d3 * a,
    };
}

C44Matrix operator/(const C44Matrix& l, float a) {
    return {
        l.a0 / a,
        l.a1 / a,
        l.a2 / a,
        l.a3 / a,

        l.b0 / a,
        l.b1 / a,
        l.b2 / a,
        l.b3 / a,

        l.c0 / a,
        l.c1 / a,
        l.c2 / a,
        l.c3 / a,

        l.d0 / a,
        l.d1 / a,
        l.d2 / a,
        l.d3 / a,
    };
}
