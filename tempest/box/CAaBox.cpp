#include "tempest/box/CAaBox.hpp"

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (hi < v ? hi : v);
}

// OFFSET: 0x984930
CAaBox CAaBox::Bounding(const C3Vector* vectors, uint32_t vectorsCount) {
    CAaBox box;
    if (vectorsCount == 0) {
        box.b.x = 0.0f;
        box.b.y = 0.0f;
        box.b.z = 0.0f;
        box.t.x = 0.0f;
        box.t.y = 0.0f;
        box.t.z = 0.0f;
        return box;
    }

    box.b = vectors[0];
    box.t = vectors[0];

    for (uint32_t i = 1; i < vectorsCount; ++i) {
        const C3Vector& v = vectors[i];
        if (v.x < box.b.x)
            box.b.x = v.x;
        if (v.y < box.b.y)
            box.b.y = v.y;
        if (v.z < box.b.z)
            box.b.z = v.z;
        if (v.x > box.t.x)
            box.t.x = v.x;
        if (v.y > box.t.y)
            box.t.y = v.y;
        if (v.z > box.t.z)
            box.t.z = v.z;
    }

    return box;
}

// OFFSET: 0x7150D0
CAaBox operator|(const CAaBox& a, const CAaBox& b) {
    CAaBox out;
    out.t = C3Vector::Max(a.t, b.t);
    out.b = C3Vector::Min(a.b, b.b);
    return out;
}

// OFFSET: 0x715130
CAaBox CAaBox::operator|=(const CAaBox& other) {
    *this = *this | other;
    return *this;
}

float CAaBox::DistanceSqXY(C2Vector& pos) {
    float clampedX;
    float diffX;
    float diffY;

    if (this->b.x > pos.x)
        clampedX = this->b.x;
    else if (this->t.x >= pos.x)
        clampedX = pos.x;
    else
        clampedX = this->t.x;

    diffX = clampedX - pos.x;

    if (this->b.y > pos.y)
        diffY = this->b.y - pos.y;
    else if (this->t.y >= pos.y)
        diffY = pos.y - pos.y;
    else
        diffY = this->t.y - pos.y;

    return diffX * diffX + diffY * diffY;
}

float CAaBox::DistanceSq(C3Vector& p) {
    const float dx = p.x - clampf(p.x, b.x, t.x);
    const float dy = p.y - clampf(p.y, b.y, t.y);
    const float dz = p.z - clampf(p.z, b.z, t.z);

    return dx * dx + dy * dy + dz * dz;
}

bool CAaBox::Intersects(CAaBox* other) {
    return this->t.x >= other->b.x
        && this->t.y >= other->b.y
        && this->t.z >= other->b.z
        && this->b.x <= other->t.x
        && this->b.y <= other->t.y
        && this->b.z <= other->t.z;
}

// OFFSET: 0x75B5B0
bool CAaBox::ContainsPoint(C3Vector& point) {
    return this->b.x <= point.x && this->b.y <= point.y && this->b.z <= point.z && this->t.x >= point.x && this->t.y >= point.y && this->t.z >= point.z;
}
