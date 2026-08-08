#include "tempest/box/CAaBox.hpp"

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
