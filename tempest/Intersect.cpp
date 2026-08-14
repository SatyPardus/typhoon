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
bool NTempest::Intersect(const C3Vector& point, const C3Vector* verts, uint32_t count, C3Vector::EAxis axis) {
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

// OFFSET: 0x9836B0
bool NTempest::Intersect(const CRay* ray, const C3Vector* verts, const int idx[3], float* outT, float* outUV, float eps) {
    const float lo = -eps;
    const float hi = 1.0f + eps;

    const C3Vector* p0 = &verts[idx[0]];
    const C3Vector* p1 = &verts[idx[1]];
    const C3Vector* p2 = &verts[idx[2]];

    const C3Vector e1 = { p1->x - p0->x, p1->y - p0->y, p1->z - p0->z };
    const C3Vector e2 = { p2->x - p0->x, p2->y - p0->y, p2->z - p0->z };

    C3Vector pv;
    pv.x = ray->dir.y * e2.z - ray->dir.z * e2.y;
    pv.y = ray->dir.z * e2.x - ray->dir.x * e2.z;
    pv.z = ray->dir.x * e2.y - ray->dir.y * e2.x;

    const float det = e1.x * pv.x + e1.y * pv.y + e1.z * pv.z;

    if (det > -0.000001f && det < 0.000001f)
        return false;

    const float invDet = 1.0f / det;

    const C3Vector tv = { ray->origin.x - p0->x,
                          ray->origin.y - p0->y,
                          ray->origin.z - p0->z };

    const float u = (tv.x * pv.x + tv.y * pv.y + tv.z * pv.z) * invDet;
    if (lo > u)
        return false;
    if (u > hi)
        return false;

    C3Vector qv;
    qv.x = tv.y * e1.z - tv.z * e1.y;
    qv.y = tv.z * e1.x - tv.x * e1.z;
    qv.z = tv.x * e1.y - tv.y * e1.x;

    const float v = (ray->dir.x * qv.x + ray->dir.y * qv.y + ray->dir.z * qv.z) * invDet;
    if (lo > v)
        return false;
    if (v + u > hi)
        return false;

    if (outT)
        *outT = (e2.x * qv.x + e2.y * qv.y + e2.z * qv.z) * invDet;

    if (outUV) {
        outUV[0] = u;
        outUV[1] = v;
    }

    return true;
}
