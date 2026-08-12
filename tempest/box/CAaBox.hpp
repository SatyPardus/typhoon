#ifndef TEMPEST_BOX_C_AABOX_HPP
#define TEMPEST_BOX_C_AABOX_HPP

#include "tempest/Vector.hpp"

class CAaBox {
    public:
    // Member variables
    C3Vector b;
    C3Vector t;

    float DistanceSqXY(C2Vector& pos);
    float DistanceSq(C3Vector& pos);
    bool Intersects(CAaBox* other);

    static CAaBox Bounding(const C3Vector* vectors, uint32_t vectorsCount);
};

#endif
