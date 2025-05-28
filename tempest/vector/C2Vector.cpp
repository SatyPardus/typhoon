#include "tempest/vector/C2Vector.hpp"
#include "tempest/Math.hpp"

#include <storm/Error.hpp>


C2Vector C2Vector::FromAxisAngle(float axang, float mag) {
    float x = mag * CMath::cos(axang);
    float y = mag * CMath::sin(axang);
    return { x, y };
}

C2Vector C2Vector::Min(const C2Vector& a, const C2Vector& b) {
    float x = std::fmin(a.x, b.x);
    float y = std::fmin(a.y, b.y);
    return { x, y };
}

C2Vector C2Vector::Max(const C2Vector& a, const C2Vector& b) {
    float x = std::fmax(a.x, b.x);
    float y = std::fmax(a.y, b.y);
    return { x, y };
}

C2Vector C2Vector::Lerp(const C2Vector& a, const C2Vector& l, const C2Vector& h) {
    return {
        l.x + (h.x - l.x) * a.x,
        l.y + (h.y - l.y) * a.y
    };
}

float C2Vector::Dot(const C2Vector& l, const C2Vector& r) {
    return (l.x * r.x) + (l.y * r.y);
}

float C2Vector::Cross(const C2Vector& l, const C2Vector& r) {
    return (l.x * r.y) - (r.x * l.y);
}

void C2Vector::Get(float& ox, float& oy) const {
    ox = this->x;
    oy = this->y;
}
void C2Vector::Set(float nx, float ny) {
    this->x = nx;
    this->y = ny;
}

C2Vector& C2Vector::operator+=(float a) {
    this->x += a;
    this->y += a;
    return *this;
}

C2Vector& C2Vector::operator+=(const C2Vector& a) {
    this->x += a.x;
    this->y += a.y;
    return *this;
}

C2Vector& C2Vector::operator-=(float a) {
    this->x -= a;
    this->y -= a;
    return *this;
}

C2Vector& C2Vector::operator-=(const C2Vector& a) {
    this->x -= a.x;
    this->y -= a.y;
    return *this;
}

C2Vector& C2Vector::operator*=(float a) {
    this->x *= a;
    this->y *= a;
    return *this;
}

C2Vector& C2Vector::operator*=(const C2Vector& a) {
    this->x *= a.x;
    this->y *= a.y;
    return *this;
}

C2Vector& C2Vector::operator/=(float a) {
    this->x /= a;
    this->y /= a;
    return *this;
}

C2Vector& C2Vector::operator/=(const C2Vector& a) {
    this->x /= a.x;
    this->y /= a.y;
    return *this;
}

C2Vector C2Vector::operator-() {
    return { -this->x, -this->y };
}

float& C2Vector::operator[](uint32_t sub) {
    STORM_ASSERT(sub < C2Vector::eComponents);
    switch (sub) {
    case 1:
        return this->y;
    case 0:
    default:
        return this->x;
    }
}

const float& C2Vector::operator[](uint32_t sub) const {
    STORM_ASSERT(sub < C2Vector::eComponents);
    switch (sub) {
    case 1:
        return this->y;
    case 0:
    default:
        return this->x;
    }
}

float C2Vector::SquaredMag() const {
    return (this->x * this->x) + (this->y * this->y);
}

float C2Vector::Mag() const {
    return CMath::sqrt(this->SquaredMag());
}

float C2Vector::SumC() const {
    return this->x + this->y;
}

bool C2Vector::IsUnit() const {
    return CMath::fequalz(this->SquaredMag(), 1.0f, 0.001f);
}

float C2Vector::AxisAngle() const {
    return this->AxisAngle(this->Mag());
}

float C2Vector::AxisAngle(float mag) const {
    if (CMath::fequal(mag, 0.0f)) {
        return 0.0f;
    }

    float angle = CMath::acos(this->x / mag);

    if (this->y >= 0.0f) {
        return angle;
    }
    else {
        return 6.2831855f - angle;
    }
}

void C2Vector::Normalize() {
    float mag = this->Mag();
    STORM_ASSERT(mag > 0.0f);
    mag = 1.0f / mag;
    this->x *= mag;
    this->y *= mag;
}

void C2Vector::Scale(float a) {
    float mag = this->Mag();
    STORM_ASSERT(mag > 0.0f);
    mag = a * 1.0f / mag;
    this->x *= mag;
    this->y *= mag;
}

bool C2Vector::operator==(const C2Vector& v) {
    return this->x == v.x && this->y == v.y;
}
