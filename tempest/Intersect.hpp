#ifndef TEMPEST_INTERSECT_HPP
#define TEMPEST_INTERSECT_HPP

#include "tempest/Vector.hpp"
#include "tempest/Plane.hpp"
#include "tempest/ray/CRay.hpp"

namespace NTempest {

bool Intersect(const C3Vector& point, const C3Vector* verts, uint32_t count, C3Vector::EAxis axis);

bool Intersect(const CRay* ray, const C3Vector* verts, uint16_t* idx, float* outT, float* outUV, float eps);

bool Intersect(CRay& ray, C4Plane& plane, float* hitT, C3Vector* hitPoint, float tollerance);

} // namespace NTempest

#endif
