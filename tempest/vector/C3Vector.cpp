#include "tempest/vector/C3Vector.hpp"
#include "tempest/Math.hpp"
#include "tempest/Matrix.hpp"

#include <storm/Error.hpp>


C3Vector C3Vector::Min(const C3Vector& a, const C3Vector& b) {
    float x = std::fmin(a.x, b.x);
    float y = std::fmin(a.y, b.y);
    float z = std::fmin(a.z, b.z);
    return { x, y, z };
}

C3Vector C3Vector::Max(const C3Vector& a, const C3Vector& b) {
    float x = std::fmax(a.x, b.x);
    float y = std::fmax(a.y, b.y);
    float z = std::fmax(a.z, b.z);
    return { x, y, z };
}

C3Vector C3Vector::Lerp(const C3Vector& a, const C3Vector& l, const C3Vector& h) {
    return {
        l.x + (h.x - l.x) * a.x,
        l.y + (h.y - l.y) * a.y,
        l.z + (h.z - l.z) * a.z
    };
}

float C3Vector::Dot(const C3Vector& l, const C3Vector& r) {
    return (l.x * r.x) + (l.y * r.y) + (l.z * r.z);
}

C3Vector C3Vector::Cross(const C3Vector& l, const C3Vector& r) {
    return {
        (l.y * r.z) - (l.z * r.y),
        (l.z * r.x) - (l.x * r.z),
        (l.x * r.y) - (l.y * r.x)
    };
}

C3Vector C3Vector::ProjectionOnPlane(const C3Vector& v, const C3Vector& normal) {
    return v - (C3Vector::Dot(v, normal) * normal);
}

C3Vector C3Vector::NearestOnPlane(const C3Vector& p, const C3Vector& onplane, const C3Vector& normal) {
    return onplane + C3Vector::ProjectionOnPlane(p - onplane, normal);
}

void C3Vector::Get(float& ox, float& oy, float& oz) const {
    ox = this->x;
    oy = this->y;
    oz = this->z;
}
void C3Vector::Set(float nx, float ny, float nz) {
    this->x = nx;
    this->y = ny;
    this->z = nz;
}

C3Vector& C3Vector::operator+=(float a) {
    this->x += a;
    this->y += a;
    this->z += a;
    return *this;
}

C3Vector& C3Vector::operator+=(const C3Vector& a) {
    this->x += a.x;
    this->y += a.y;
    this->z += a.z;
    return *this;
}

C3Vector& C3Vector::operator-=(float a) {
    this->x -= a;
    this->y -= a;
    this->z -= a;
    return *this;
}

C3Vector& C3Vector::operator-=(const C3Vector& a) {
    this->x -= a.x;
    this->y -= a.y;
    this->z -= a.z;
    return *this;
}

C3Vector& C3Vector::operator*=(float a) {
    this->x *= a;
    this->y *= a;
    this->z *= a;
    return *this;
}

C3Vector& C3Vector::operator*=(const C3Vector& a) {
    this->x *= a.x;
    this->y *= a.y;
    this->z *= a.z;
    return *this;
}

C3Vector& C3Vector::operator/=(float a) {
    this->x /= a;
    this->y /= a;
    this->z /= a;
    return *this;
}

C3Vector& C3Vector::operator/=(const C3Vector& a) {
    this->x /= a.x;
    this->y /= a.y;
    this->z /= a.z;
    return *this;
}

C3Vector C3Vector::operator-() {
    return { -this->x, -this->y, -this->z };
}

float& C3Vector::operator[](uint32_t sub) {
    STORM_ASSERT(sub < C3Vector::eComponents);
    switch (sub) {
    case C3AXIS_Z:
        return this->z;
    case C3AXIS_Y:
        return this->y;
    case C3AXIS_X:
    default:
        return this->x;
    }
}

const float& C3Vector::operator[](uint32_t sub) const {
    STORM_ASSERT(sub < C3Vector::eComponents);
    switch (sub) {
    case C3AXIS_Z:
        return this->z;
    case C3AXIS_Y:
        return this->y;
    case C3AXIS_X:
    default:
        return this->x;
    }
}

float C3Vector::SquaredMag() const {
    return this->x * this->x + this->y * this->y + this->z * this->z;
}

float C3Vector::Mag() const {
    return CMath::sqrt(this->SquaredMag());
}

float C3Vector::SumC() const {
    return this->x + this->y + this->z;
}

bool C3Vector::IsUnit() const {
    return CMath::fequalz(this->SquaredMag(), 1.0f, 0.001f);
}

void C3Vector::Normalize() {
    float mag = this->Mag();
    STORM_ASSERT(mag > 0.0f);
    mag = 1.0f / mag;
    this->x *= mag;
    this->y *= mag;
    this->z *= mag;
}

void C3Vector::Scale(float a) {
    float mag = this->Mag();
    STORM_ASSERT(mag > 0.0f);
    mag = a * 1.0f / mag;
    this->x *= mag;
    this->y *= mag;
    this->z *= mag;
}

C3Vector::EAxis C3Vector::MajorAxis() const {
    float fx = CMath::fabs(this->x);
    float fy = CMath::fabs(this->y);
    float fz = CMath::fabs(this->z);

    if (fx <= fy) {
        if (fy > fz) {
            return C3AXIS_Y;
        }
    } else if (fx > fz) {
        return C3AXIS_X;
    }
    return C3AXIS_Z;
}

C3Vector::EAxis C3Vector::MinorAxis() const {
    float fx = CMath::fabs(this->x);
    float fy = CMath::fabs(this->y);
    float fz = CMath::fabs(this->z);

    if (fx >= fy) {
        if (fz > fy) {
            return C3AXIS_Y;
        }
    } else if (fx >= fz) {
        return C3AXIS_Z;
    } else {
        return C3AXIS_X;
    }
    return C3AXIS_Z;
}

C3Vector operator+(const C3Vector& l, const C3Vector& r) {
    float x = l.x + r.x;
    float y = l.y + r.y;
    float z = l.z + r.z;

    return { x, y, z };
}

C3Vector operator-(const C3Vector& l, const C3Vector& r) {
    float x = l.x - r.x;
    float y = l.y - r.y;
    float z = l.z - r.z;

    return { x, y, z };
}

C3Vector operator*(const C3Vector& l, float r) {
    return { l.x * r, l.y * r, l.z * r };
}

C3Vector operator*(float l, const C3Vector& r) {
    return { l * r.x, l * r.y, l * r.z };
}

C3Vector operator*(const C3Vector& l, const C44Matrix& r) {
    float x = r.c0 * l.z + r.b0 * l.y + r.a0 * l.x + r.d0;
    float y = r.c1 * l.z + r.b1 * l.y + r.a1 * l.x + r.d1;
    float z = r.c2 * l.z + r.b2 * l.y + r.a2 * l.x + r.d2;

    return { x, y, z };
}

bool operator==(const C3Vector& l, const C3Vector& r) {
    return l.x == r.x && l.y == r.y && l.z == r.z;
}

bool operator!=(const C3Vector& l, const C3Vector& r) {
    return l.x != r.x || l.y != r.y || l.z != r.z;
}
