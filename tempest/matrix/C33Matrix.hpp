#ifndef TEMPEST_MATRIX_C_33MATRIX_HPP
#define TEMPEST_MATRIX_C_33MATRIX_HPP

#include <cstdint>

class C2Vector;
class C3Vector;
class C34Matrix;
class C44Matrix;
class C4Quaternion;

class C33Matrix {
    public:
    enum : uint32_t {
        eComponents = 9
    };

    // Static functions
    static float Det(float a, float b, float c, float d);
    static C33Matrix RotationAroundX(float angle);
    static C33Matrix RotationAroundY(float angle);
    static C33Matrix RotationAroundZ(float angle);

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
    C33Matrix(const C3Vector& r0, const C3Vector& r1, const C3Vector& r2);
    explicit C33Matrix(const C34Matrix& m);
    explicit C33Matrix(const C44Matrix& m);
    explicit C33Matrix(const C4Quaternion& rotation);
    C33Matrix(float a0, float a1, float a2, float b0, float b1, float b2, float c0, float c1, float c2);
    explicit C33Matrix(float a);

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

    bool ToEulerAnglesXYZ(float& xa_, float& ya_, float& za_);
    bool ToEulerAnglesXZY(float& xa_, float& za_, float& ya_);
    bool ToEulerAnglesYXZ(float& ya_, float& xa_, float& za_);
    bool ToEulerAnglesYZX(float& ya_, float& za_, float& xa_);
    bool ToEulerAnglesZXY(float& za_, float& xa_, float& ya_);
    bool ToEulerAnglesZYX(float& za_, float& ya_, float& xa_);
    void FromEulerAnglesXYZ(float yaw, float pitch, float roll);
    void FromEulerAnglesXZY(float yaw, float pitch, float roll);
    void FromEulerAnglesYXZ(float yaw, float pitch, float roll);
    void FromEulerAnglesYZX(float yaw, float pitch, float roll);
    void FromEulerAnglesZXY(float yaw, float pitch, float roll);
    void FromEulerAnglesZYX(float yaw, float pitch, float roll);
};

C33Matrix operator*(const C33Matrix& l, const C33Matrix& r);
C33Matrix operator*(const C33Matrix& l, float a);

C33Matrix operator/(const C33Matrix& l, float a);

#endif
