#ifndef TEMPEST_VECTOR_C_2VECTOR_HPP
#define TEMPEST_VECTOR_C_2VECTOR_HPP

#include <cstdint>

class C2Vector {
    public:
    enum : uint32_t {
        eComponents = 2
    };

    // Static functions
    static C2Vector FromAxisAngle(float axang, float mag);
    static C2Vector Min(const C2Vector& a, const C2Vector& b);
    static C2Vector Max(const C2Vector& a, const C2Vector& b);
    static C2Vector Lerp(const C2Vector& a, const C2Vector& l, const C2Vector& h);
    static float Dot(const C2Vector& l, const C2Vector& r);
    static float Cross(const C2Vector& l, const C2Vector& r);

    // Member variables
    float x = 0.0f;
    float y = 0.0f;

    // Member functions
    C2Vector() = default;
    C2Vector(float x, float y)
        : x(x)
        , y(y) {};
    explicit C2Vector(float a)
        : x(a)
        , y(a) {};

    void Get(float& ox, float& oy) const;
    void Set(float nx, float ny);

    C2Vector& operator+=(float a);
    C2Vector& operator+=(const C2Vector& a);
    C2Vector& operator-=(float a);
    C2Vector& operator-=(const C2Vector& a);
    C2Vector& operator*=(float a);
    C2Vector& operator*=(const C2Vector& a);
    C2Vector& operator/=(float a);
    C2Vector& operator/=(const C2Vector& a);

    C2Vector operator-();

    float& operator[](uint32_t sub);
    const float& operator[](uint32_t sub) const;

    float SquaredMag() const;
    float Mag() const;
    float SumC() const;
    bool IsUnit() const;
    float AxisAngle() const;
    float AxisAngle(float mag) const;

    void Normalize();
    void Scale(float a);

    bool operator==(const C2Vector& v);
};

#endif
