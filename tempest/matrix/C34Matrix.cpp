#include "tempest/matrix/C34Matrix.hpp"
#include "tempest/vector/C3Vector.hpp"
#include "tempest/matrix/C33Matrix.hpp"
#include "tempest/quaternion/C4Quaternion.hpp"


C34Matrix C34Matrix::Rotation(float angle, const C3Vector& axis, bool unit) {
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
    C34Matrix result;

    result.a0 = axis_.x * axis_.x * one_c + cosa;
    result.a1 = axis_.x * axis_.y * one_c + zs;
    result.a2 = axis_.x * axis_.z * one_c - ys;

    result.b0 = axis_.x * axis_.y * one_c - zs;
    result.b1 = axis_.y * axis_.y * one_c + cosa;
    result.b2 = axis_.y * axis_.z * one_c + xs;

    result.c0 = axis_.x * axis_.z * one_c + ys;
    result.c1 = axis_.y * axis_.z * one_c - xs;
    result.c2 = axis_.z * axis_.z * one_c + cosa;

    result.d0 = 0.0f;
    result.d1 = 0.0f;
    result.d2 = 0.0f;

    return result;
}

C34Matrix::C34Matrix(const C3Vector& r0, const C3Vector& r1, const C3Vector& r2, const C3Vector& r3)
    : a0(r0.x)
    , a1(r0.y)
    , a2(r0.z)
    , b0(r1.x)
    , b1(r1.y)
    , b2(r1.z)
    , c0(r2.x)
    , c1(r2.y)
    , c2(r2.z)
    , d0(r3.x)
    , d1(r3.y)
    , d2(r3.z) {
}

C34Matrix::C34Matrix(const C3Vector& r0, const C3Vector& r1, const C3Vector& r2)
    : a0(r0.x)
    , a1(r0.y)
    , a2(r0.z)
    , b0(r1.x)
    , b1(r1.y)
    , b2(r1.z)
    , c0(r2.x)
    , c1(r2.y)
    , c2(r2.z)
    , d0(0.0f)
    , d1(0.0f)
    , d2(0.0f) {
}

C34Matrix::C34Matrix(const C33Matrix& m)
    : a0(m.a0)
    , a1(m.a1)
    , a2(m.a2)
    , b0(m.b0)
    , b1(m.b1)
    , b2(m.b2)
    , c0(m.c0)
    , c1(m.c1)
    , c2(m.c2)
    , d0(0.0f)
    , d1(0.0f)
    , d2(0.0f) {
}

C34Matrix::C34Matrix(float a0, float a1, float a2, float b0, float b1, float b2, float c0, float c1, float c2, float d0, float d1, float d2)
    : a0(a0)
    , a1(a1)
    , a2(a2)
    , b0(b0)
    , b1(b1)
    , b2(b2)
    , c0(c0)
    , c1(c1)
    , c2(c2)
    , d0(d0)
    , d1(d1)
    , d2(d2) {
}

C34Matrix::C34Matrix(float a)
    : a0(a)
    , a1(a)
    , a2(a)
    , b0(a)
    , b1(a)
    , b2(a)
    , c0(a)
    , c1(a)
    , c2(a)
    , d0(a)
    , d1(a)
    , d2(a) {
}

C34Matrix& C34Matrix::operator+=(const C34Matrix& a) {
    this->a0 += a.a0;
    this->a1 += a.a1;
    this->a2 += a.a2;

    this->b0 += a.b0;
    this->b1 += a.b1;
    this->b2 += a.b2;

    this->c0 += a.c0;
    this->c1 += a.c1;
    this->c2 += a.c2;

    this->d0 += a.d0;
    this->d1 += a.d1;
    this->d2 += a.d2;

    return *this;
}

C34Matrix& C34Matrix::operator-=(const C34Matrix& a) {
    this->a0 -= a.a0;
    this->a1 -= a.a1;
    this->a2 -= a.a2;

    this->b0 -= a.b0;
    this->b1 -= a.b1;
    this->b2 -= a.b2;

    this->c0 -= a.c0;
    this->c1 -= a.c1;
    this->c2 -= a.c2;

    this->d0 -= a.d0;
    this->d1 -= a.d1;
    this->d2 -= a.d2;

    return *this;
}

C34Matrix& C34Matrix::operator*=(float a) {
    this->a0 *= a;
    this->a1 *= a;
    this->a2 *= a;

    this->b0 *= a;
    this->b1 *= a;
    this->b2 *= a;

    this->c0 *= a;
    this->c1 *= a;
    this->c2 *= a;

    this->d0 *= a;
    this->d1 *= a;
    this->d2 *= a;

    return *this;
}

C34Matrix& C34Matrix::operator*=(const C34Matrix& a) {
    *this = *this * a;
    return *this;
}

C34Matrix& C34Matrix::operator/=(float a) {
    this->a0 /= a;
    this->a1 /= a;
    this->a2 /= a;

    this->b0 /= a;
    this->b1 /= a;
    this->b2 /= a;

    this->c0 /= a;
    this->c1 /= a;
    this->c2 /= a;

    this->d0 /= a;
    this->d1 /= a;
    this->d2 /= a;

    return *this;
}

void C34Matrix::Zero() {
    this->a0 = 0.0f;
    this->a1 = 0.0f;
    this->a2 = 0.0f;

    this->b0 = 0.0f;
    this->b1 = 0.0f;
    this->b2 = 0.0f;

    this->c0 = 0.0f;
    this->c1 = 0.0f;
    this->c2 = 0.0f;

    this->d0 = 0.0f;
    this->d1 = 0.0f;
    this->d2 = 0.0f;
}

void C34Matrix::Identity() {
    this->a0 = 1.0f;
    this->a1 = 0.0f;
    this->a2 = 0.0f;

    this->b0 = 0.0f;
    this->b1 = 1.0f;
    this->b2 = 0.0f;

    this->c0 = 0.0f;
    this->c1 = 0.0f;
    this->c2 = 1.0f;

    this->d0 = 0.0f;
    this->d1 = 0.0f;
    this->d2 = 0.0f;
}

float C34Matrix::Trace() {
    return this->a0 + this->b1 + this->c2;
}

void C34Matrix::Scale(float scale) {
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

void C34Matrix::Scale(const C3Vector& scale) {
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

void C34Matrix::Rotate(const C4Quaternion& rotation) {
    *this = C34Matrix(C33Matrix(rotation)) * (*this);
}

void C34Matrix::Rotate(float angle, const C3Vector& axis, bool unit) {
    *this = C34Matrix::Rotation(angle, axis, unit) * (*this);
}

void C34Matrix::Translate(const C3Vector& move) {
    this->d0 = this->a0 * move.x + this->b0 * move.y + this->c0 * move.z + this->d0;
    this->d1 = this->a1 * move.x + this->b1 * move.y + this->c1 * move.z + this->d1;
    this->d2 = this->a2 * move.x + this->b2 * move.y + this->c2 * move.z + this->d2;
}

C34Matrix C34Matrix::AffineInverse(const C3Vector& v) const {
    C33Matrix rotationScale(*this);
    C3Vector s = { 1.0f / v.x, 1.0f / v.y, 1.0f / v.z };
    rotationScale.Scale(s);
    C34Matrix matrix(rotationScale.Transpose());
    matrix.Scale(s);
    C3Vector move(-this->d0, -this->d1, -this->d2);
    matrix.Translate(move);
    return matrix;
}

C34Matrix C34Matrix::AffineInverse(float a) const {
    if (CMath::fequalz(a, 1.0f, 0.00000095367432f)) {
        return this->AffineInverse();
    }

    C34Matrix matrix(C33Matrix(*this).Transpose());
    matrix.Scale(1.0f / (a * a));
    C3Vector move(-this->d0, -this->d1, -this->d2);
    matrix.Translate(move);
    return matrix;
}

C34Matrix C34Matrix::AffineInverse() const {
    C34Matrix matrix(C33Matrix(*this).Transpose());
    C3Vector move(-this->d0, -this->d1, -this->d2);
    matrix.Translate(move);
    return matrix;
}

C34Matrix operator+(const C34Matrix& l, const C34Matrix& r) {
    return {
        l.a0 + r.a0,
        l.a1 + r.a1,
        l.a2 + r.a2,

        l.b0 + r.b0,
        l.b1 + r.b1,
        l.b2 + r.b2,

        l.c0 + r.c0,
        l.c1 + r.c1,
        l.c2 + r.c2,

        l.d0 + r.d0,
        l.d1 + r.d1,
        l.d2 + r.d2,
    };
}

C34Matrix operator+(const C34Matrix& l, float a) {
    return {
        l.a0 + a,
        l.a1 + a,
        l.a2 + a,

        l.b0 + a,
        l.b1 + a,
        l.b2 + a,

        l.c0 + a,
        l.c1 + a,
        l.c2 + a,

        l.d0 + a,
        l.d1 + a,
        l.d2 + a,
    };
}

C34Matrix operator-(const C34Matrix& l, const C34Matrix& r) {
    return {
        l.a0 - r.a0,
        l.a1 - r.a1,
        l.a2 - r.a2,

        l.b0 - r.b0,
        l.b1 - r.b1,
        l.b2 - r.b2,

        l.c0 - r.c0,
        l.c1 - r.c1,
        l.c2 - r.c2,

        l.d0 - r.d0,
        l.d1 - r.d1,
        l.d2 - r.d2,
    };
}

C34Matrix operator-(const C34Matrix& l, float a) {
    return {
        l.a0 - a,
        l.a1 - a,
        l.a2 - a,

        l.b0 - a,
        l.b1 - a,
        l.b2 - a,

        l.c0 - a,
        l.c1 - a,
        l.c2 - a,

        l.d0 - a,
        l.d1 - a,
        l.d2 - a,
    };
}

C34Matrix operator*(const C34Matrix& l, const C34Matrix& r) {
    float d2 = (((l.d0 * r.a2) + (l.d1 * r.b2)) + (l.d2 * r.c2)) + r.d2;
    float d1 = (((l.d0 * r.a1) + (l.d1 * r.b1)) + (l.d2 * r.c1)) + r.d1;
    float d0 = (((l.d0 * r.a0) + (l.d1 * r.b0)) + (l.d2 * r.c0)) + r.d0;

    float c2 = ((l.c0 * r.a2) + (l.c1 * r.b2)) + (l.c2 * r.c2);
    float c1 = ((l.c0 * r.a1) + (l.c1 * r.b1)) + (l.c2 * r.c1);
    float c0 = ((l.c0 * r.a0) + (l.c1 * r.b0)) + (l.c2 * r.c0);

    float b2 = ((l.b0 * r.a2) + (l.b1 * r.b2)) + (l.b2 * r.c2);
    float b1 = ((l.b0 * r.a1) + (l.b1 * r.b1)) + (l.b2 * r.c1);
    float b0 = ((l.b0 * r.a0) + (l.b1 * r.b0)) + (l.b2 * r.c0);

    float a2 = ((l.a0 * r.a2) + (l.a1 * r.b2)) + (l.a2 * r.c2);
    float a1 = ((l.a0 * r.a1) + (l.a1 * r.b1)) + (l.a2 * r.c1);
    float a0 = ((l.a0 * r.a0) + (l.a1 * r.b0)) + (l.a2 * r.c0);

    return { a0, a1, a2, b0, b1, b2, c0, c1, c2, d0, d1, d2 };
}

C34Matrix operator*(const C34Matrix& l, float a) {
    return {
        l.a0 * a,
        l.a1 * a,
        l.a2 * a,

        l.b0 * a,
        l.b1 * a,
        l.b2 * a,

        l.c0 * a,
        l.c1 * a,
        l.c2 * a,

        l.d0 * a,
        l.d1 * a,
        l.d2 * a,
    };
}

C34Matrix operator/(const C34Matrix& l, float a) {
    return {
        l.a0 / a,
        l.a1 / a,
        l.a2 / a,

        l.b0 / a,
        l.b1 / a,
        l.b2 / a,

        l.c0 / a,
        l.c1 / a,
        l.c2 / a,

        l.d0 / a,
        l.d1 / a,
        l.d2 / a,
    };
}
