#ifndef TEMPEST_MATRIX_C_34MATRIX_HPP
#define TEMPEST_MATRIX_C_34MATRIX_HPP

#include <cstdint>

class C2Vector;
class C3Vector;
class C33Matrix;
class C4Quaternion;

class C34Matrix {
    public:
    enum : uint32_t {
        eComponents = 12
    };

    // Static functions
    static C34Matrix Rotation(float angle, const C3Vector& axis, bool unit);

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
    float d0 = 0.0f;
    float d1 = 0.0f;
    float d2 = 0.0f;

    // Member functions
    C34Matrix() = default;
    C34Matrix(const C3Vector& r0, const C3Vector& r1, const C3Vector& r2, const C3Vector& r3);
    C34Matrix(const C3Vector& r0, const C3Vector& r1, const C3Vector& r2);
    explicit C34Matrix(const C33Matrix& m);
    C34Matrix(float a0, float a1, float a2, float b0, float b1, float b2, float c0, float c1, float c2, float d0, float d1, float d2);
    explicit C34Matrix(float a);

    C34Matrix& operator+=(const C34Matrix& a);
    C34Matrix& operator-=(const C34Matrix& a);
    C34Matrix& operator*=(float a);
    C34Matrix& operator*=(const C34Matrix& a);
    C34Matrix& operator/=(float a);

    void Zero();
    void Identity();
    float Trace();
    void Scale(float scale);
    void Scale(const C3Vector& scale);
    void Rotate(const C4Quaternion& rotation);
    void Rotate(float angle, const C3Vector& axis, bool unit);
    void Translate(const C3Vector& move);
    C34Matrix AffineInverse(const C3Vector& v) const;
    C34Matrix AffineInverse(float a) const;
    C34Matrix AffineInverse() const;
};

C34Matrix operator+(const C34Matrix& l, const C34Matrix& r);
C34Matrix operator+(const C34Matrix& l, float a);

C34Matrix operator-(const C34Matrix& l, const C34Matrix& r);
C34Matrix operator-(const C34Matrix& l, float a);

C34Matrix operator*(const C34Matrix& l, const C34Matrix& r);
C34Matrix operator*(const C34Matrix& l, float a);

C34Matrix operator/(const C34Matrix& l, float a);

#endif
