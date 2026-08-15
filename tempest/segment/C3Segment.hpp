#ifndef TEMPEST_VECTOR_C_3SEGMENT_HPP
#define TEMPEST_VECTOR_C_3SEGMENT_HPP

#include <cstdint>
#include "tempest/vector/C3Vector.hpp"

class C3Segment {
    public:
    C3Vector b; // +0x00  start
    C3Vector t; // +0x0C  end

    C3Segment() {}
    C3Segment(const C3Vector& bIn, const C3Vector& tIn) // 0x0078F480
    {
        b = bIn;
        t = tIn;
    }

    // OFFSET: 0x78F4D0
    C3Vector Lerp(float s) const {
        C3Vector r;
        r.x = b.x + (t.x - b.x) * s;
        r.y = b.y + (t.y - b.y) * s;
        r.z = b.z + (t.z - b.z) * s;
        return r;
    }
};

#endif
