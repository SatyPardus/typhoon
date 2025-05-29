#ifndef TEMPEST_MATRIX_C_33MATRIX_HPP
#define TEMPEST_MATRIX_C_33MATRIX_HPP

#include "tempest/vector/C2Vector.hpp"
#include "tempest/vector/C3Vector.hpp"

class C4Quaternion;

class C33Matrix {
    public:
    enum : uint32_t {
        eComponents = 9
    };

    // Static functions
    static float Det(float a, float b, float c, float d);
    static C33Matrix Rotation(float angle);
    static C33Matrix Rotation(float angle, const C3Vector& axis, bool unit);

    // Member variables
    float a0 = 1.0f;
    float a1 = 0.0f;
    float a2 = 0.0f;
    float b0 = 0.0f;
    float b1 = 1.0f;
    float b2 = 0.0f;
    float c0 = 0.0f;
    float c1 = 0.0f;
    float c2 = 1.0f;

    // Member functions
    C33Matrix() = default;
    C33Matrix(const C3Vector& r0, const C3Vector& r1, const C3Vector& r2)
        : a0(r0.x)
        , a1(r0.y)
        , a2(r0.z)
        , b0(r1.x)
        , b1(r1.y)
        , b2(r1.z)
        , c0(r2.x)
        , c1(r2.y)
        , c2(r2.z) {};
    C33Matrix(float a0, float a1, float a2, float b0, float b1, float b2, float c0, float c1, float c2)
        : a0(a0)
        , a1(a1)
        , a2(a2)
        , b0(b0)
        , b1(b1)
        , b2(b2)
        , c0(c0)
        , c1(c1)
        , c2(c2) {};
    explicit C33Matrix(float a)
        : a0(a)
        , a1(a)
        , a2(a)
        , b0(a)
        , b1(a)
        , b2(a)
        , c0(a)
        , c1(a)
        , c2(a) {};

    C33Matrix& operator+=(const C33Matrix& a);
    C33Matrix& operator-=(const C33Matrix& a);
    C33Matrix& operator*=(float a);
    C33Matrix& operator*=(const C33Matrix& a);
    C33Matrix& operator/=(float a);

    void Zero();
    void Identity();
    float Trace();
    void Scale(float scale);
    void Scale(float x, float y);
    void Scale(const C2Vector& scale);
    void Scale(float x, float y, float z);
    void Scale(const C3Vector& scale);
    void Rotate(float angle);
    void Rotate(const C4Quaternion& rotation);
    void Rotate(float angle, const C3Vector& axis, bool unit);
    void Translate(const C2Vector& move);
    C33Matrix Transpose() const;
    float Determinant() const;
    C33Matrix Cofactors() const;
    C33Matrix Adjoint() const;
    C33Matrix Inverse() const;
    C33Matrix Inverse(float det) const;
    C33Matrix AffineInverse(const C3Vector& v) const;
    C33Matrix AffineInverse(float a) const;
};

C33Matrix operator*(const C33Matrix& l, float a);
C33Matrix operator*(const C33Matrix& l, const C33Matrix& r);
C33Matrix operator/(const C33Matrix& l, float a);

#endif
