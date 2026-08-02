#include "tempest/vector/C4Vector.hpp"

C4Vector operator*(const C4Vector& l, const C44Matrix& r) {
    float x = r.a0 * l.x + r.b0 * l.y + r.c0 * l.z + r.d0 * l.w;
    float y = r.a1 * l.x + r.b1 * l.y + r.c1 * l.z + r.d1 * l.w;
    float z = r.a2 * l.x + r.b2 * l.y + r.c2 * l.z + r.d2 * l.w;
    float w = r.a3 * l.x + r.b3 * l.y + r.c3 * l.z + r.d3 * l.w;
    return { x, y, z, w };
}
