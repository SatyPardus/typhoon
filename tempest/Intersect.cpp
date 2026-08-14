#include "tempest/Intersect.hpp"

namespace {
    struct ProjectedAxes {
        uint32_t u;
        uint32_t v;
    };

    constexpr ProjectedAxes PROJECTED_AXES[3] = {
        { 1, 2 }, // C3AXIS_X
        { 2, 0 }, // C3AXIS_Y
        { 0, 1 }  // C3AXIS_Z
    };

    inline float Component(const C3Vector& v, uint32_t index) {
        return (&v.x)[index];
    }
}

// OFFSET: 0x9830D0
bool Intersect(const C3Vector& point, const C3Vector* verts, uint32_t count, C3Vector::EAxis axis) {
    if (count < 3) {
        return false;
    }

    uint32_t u = PROJECTED_AXES[axis].u;
    uint32_t v = PROJECTED_AXES[axis].v;

    float pu = Component(point, u);
    float pv = Component(point, v);

    bool inside = false;

    uint32_t j = count - 1;
    bool prev = pv <= Component(verts[j], v);

    for (uint32_t i = 0; i < count; j = i++) {
        float iu = Component(verts[i], u);
        float iv = Component(verts[i], v);

        bool cur = pv <= iv;

        if (cur != prev) {
            float ju = Component(verts[j], u);
            float jv = Component(verts[j], v);

            bool side = (iu - pu) * (jv - iv) <= (ju - iu) * (iv - pv);

            if (side == cur) {
                inside = !inside;
            }
        }

        prev = cur;
    }

    return inside;
}
