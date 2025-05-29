#ifndef TEMPEST_MATRIX_C_44MATRIX_HPP
#define TEMPEST_MATRIX_C_44MATRIX_HPP

#include "tempest/vector/C4Vector.hpp"
#include "tempest/matrix/C33Matrix.hpp"

class C3Vector;
class C4Quaternion;

class C44Matrix {
    public:
    enum : uint32_t {
        eComponents = 16
    };

    // Static functions
    static float Det(float a, float b, float c, float d, float e, float f, float g, float h, float i);
    static C44Matrix Rotation(float angle, const C3Vector& axis, bool unit);
    static C44Matrix RotationAroundZ(float angle);

    // Member variables
    float a0 = 1.0f;
    float a1 = 0.0f;
    float a2 = 0.0f;
    float a3 = 0.0f;
    float b0 = 0.0f;
    float b1 = 1.0f;
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

    // Member functions
    C44Matrix() = default;
    C44Matrix(const C4Vector& r0, const C4Vector& r1, const C4Vector& r2, const C4Vector& r3)
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
        , d3(r3.w) {};
    explicit C44Matrix(const C33Matrix& m)
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
        , d3(1.0f) {};
    C44Matrix(float a0, float a1, float a2, float a3, float b0, float b1, float b2, float b3, float c0, float c1, float c2, float c3, float d0, float d1, float d2, float d3)
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
        , d3(d3) {};
    explicit C44Matrix(float a)
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
        , d3(a) {};
    C44Matrix(const C4Quaternion& rotation);

    C44Matrix& operator+=(const C44Matrix& a);
    C44Matrix& operator-=(const C44Matrix& a);
    C44Matrix& operator*=(float a);
    C44Matrix& operator*=(const C44Matrix& a);
    C44Matrix& operator/=(float a);

    void Zero();
    void Identity();
    float Trace();
    void Translate(const C3Vector& move);
    void Scale(float scale);
    void Scale(const C3Vector& scale);
    void Rotate(const C4Quaternion& rotation);
    void Rotate(float angle, const C3Vector& axis, bool unit);
    C44Matrix Transpose() const;
    float Determinant() const;
    C44Matrix Cofactors() const;
    C44Matrix Adjoint() const;
    C44Matrix Inverse() const;
    C44Matrix Inverse(float det) const;
    C44Matrix AffineInverse(const C3Vector& v) const;
    C44Matrix AffineInverse(float a) const;
    C44Matrix AffineInverse() const;

    void RotateAroundZ(float angle);
};

C44Matrix operator*(const C44Matrix& l, float a);

C44Matrix operator*(const C44Matrix& l, const C44Matrix& r);

#endif
