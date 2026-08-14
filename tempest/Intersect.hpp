#ifndef TEMPEST_INTERSECT_HPP
#define TEMPEST_INTERSECT_HPP

#include "tempest/Vector.hpp"

bool Intersect(const C3Vector& point, const C3Vector* verts, uint32_t count, C3Vector::EAxis axis);

#endif
