#ifndef TEMPEST_FACET_C_FACET_HPP
#define TEMPEST_FACET_C_FACET_HPP

#include "tempest/vector/C3Vector.hpp"
#include "tempest/plane/C4Plane.hpp"

class CFacet {
    public:
    C4Plane plane;
    C3Vector v[3];

    CFacet(C3Vector& v1, C3Vector& v2, C3Vector& v3);
};

#endif
