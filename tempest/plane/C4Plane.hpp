#ifndef TEMPEST_PLANE_C_4PLANE_HPP
#define TEMPEST_PLANE_C_4PLANE_HPP

#include "tempest/Vector.hpp"

class C4Plane {
    public:
    // Member variables
    C3Vector n = { 0.0f, 0.0f, 1.0f };
    float d = 0.0f;

    C4Plane() = default;
};

#endif
