#ifndef TEMPEST_BOX_C_AABOX_HPP
#define TEMPEST_BOX_C_AABOX_HPP

#include "tempest/Vector.hpp"

class CAaBox {
    public:
    // Member variables
    C3Vector b;
    C3Vector t;

    static CAaBox Bounding(const C3Vector* vectors, uint32_t vectorsCount);
};

#endif
