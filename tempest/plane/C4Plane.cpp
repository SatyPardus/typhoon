#include "tempest/plane/C4Plane.hpp"
#include <cmath>

// OFFSET: 0x7912C0
void C4Plane::From3Pos(const C3Vector& a, const C3Vector& b, const C3Vector& c) {
    C3Vector ab = b - a;
    C3Vector ac = c - a;

    n.x = ab.y * ac.z - ab.z * ac.y;
    n.y = ab.z * ac.x - ac.z * ab.x;
    n.z = ac.y * ab.x - ac.x * ab.y;

    float invLen = 1.0f / sqrtf(n.x * n.x + n.y * n.y + n.z * n.z);
    n.x *= invLen;
    n.y *= invLen;
    n.z *= invLen;

    d = -(n.x * a.x + n.y * a.y + n.z * a.z);
}
