#ifndef TEMPEST_MATRIX_C_44MATRIX_HPP
#define TEMPEST_MATRIX_C_44MATRIX_HPP

#include <cstdint>

class C3Vector;
class C4Vector;
class C33Matrix;
class C4Quaternion;

class C44Matrix {
    public:
    enum : uint32_t {
        eComponents = 16
    };

    // Static functions
    static float Det(float a, float b, float c, float d, float e, float f, float g, float h, float i);
    static C44Matrix RotationAroundZ(float angle);
    static C44Matrix Rotation(float angle, const C3Vector& axis, bool unit);

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
    C44Matrix(const C4Vector& r0, const C4Vector& r1, const C4Vector& r2, const C4Vector& r3);
    explicit C44Matrix(const C33Matrix& m);
    C44Matrix(const C4Quaternion& rotation);
    C44Matrix(float a0, float a1, float a2, float a3, float b0, float b1, float b2, float b3, float c0, float c1, float c2, float c3, float d0, float d1, float d2, float d3);
    explicit C44Matrix(float a);

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
    void RotateAroundZ(float angle);
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
};

C44Matrix operator*(const C44Matrix& l, float a);
C44Matrix operator*(const C44Matrix& l, const C44Matrix& r);

#endif
