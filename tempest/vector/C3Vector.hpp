#ifndef TEMPEST_VECTOR_C_3VECTOR_HPP
#define TEMPEST_VECTOR_C_3VECTOR_HPP

#include "tempest/vector/C2Vector.hpp"
#include "tempest/vector/CImVector.hpp"

class C33Matrix;
class C44Matrix;

class C3Vector {
    public:
    enum : uint32_t {
        eComponents = 3
    };

    enum EAxis {
        C3AXIS_X = 0,
        C3AXIS_Y = 1,
        C3AXIS_Z = 2
    };

    // Static functions
    static C3Vector Min(const C3Vector& a, const C3Vector& b);
    static C3Vector Max(const C3Vector& a, const C3Vector& b);
    static C3Vector Lerp(const C3Vector& a, const C3Vector& l, const C3Vector& h);
    static float Dot(const C3Vector& l, const C3Vector& r);
    static C3Vector Cross(const C3Vector& l, const C3Vector& r);
    static C3Vector ProjectionOnPlane(const C3Vector& v, const C3Vector& normal);
    static C3Vector NearestOnPlane(const C3Vector& p, const C3Vector& onplane, const C3Vector& normal);

    // Member variables
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    // Member functions
    C3Vector() = default;
    C3Vector(const C2Vector& a)
        : x(a.x)
        , y(a.y)
        , z(0.0f) {};
    C3Vector(const CImVector& color)
        : x(color.r / 255.0f)
        , y(color.g / 255.0f)
        , z(color.b / 255.0f) {};
    C3Vector(float x, float y, float z)
        : x(x)
        , y(y)
        , z(z) {};
    explicit C3Vector(float a)
        : x(a)
        , y(a)
        , z(a) {};

    void Get(float& ox, float& oy, float& oz) const;
    void Set(float nx, float ny, float nz);

    C3Vector& operator+=(float a);
    C3Vector& operator+=(const C3Vector& a);
    C3Vector& operator-=(float a);
    C3Vector& operator-=(const C3Vector& a);
    C3Vector& operator*=(float a);
    C3Vector& operator*=(const C3Vector& a);
    C3Vector& operator/=(float a);
    C3Vector& operator/=(const C3Vector& a);

    C3Vector operator-() const;

    float& operator[](uint32_t sub);
    const float& operator[](uint32_t sub) const;

    float SquaredMag() const;
    float Mag() const;
    float SumC() const;
    bool IsUnit() const;

    void Normalize();
    void Scale(float a);

    EAxis MajorAxis() const;
    EAxis MinorAxis() const;
};

C3Vector operator+(const C3Vector& l, const C3Vector& r);
C3Vector operator-(const C3Vector& l, const C3Vector& r);

C3Vector operator*(const C3Vector& l, float r);
C3Vector operator*(float l, const C3Vector& r);
C3Vector operator*(const C3Vector& l, const C33Matrix& r);
C3Vector operator*(const C3Vector& l, const C44Matrix& r);

bool operator==(const C3Vector& l, const C3Vector& r);
bool operator!=(const C3Vector& l, const C3Vector& r);

#endif
