#ifndef TEMPEST_VECTOR_C_4VECTOR_HPP
#define TEMPEST_VECTOR_C_4VECTOR_HPP

#include "tempest/Matrix.hpp"

class C3Vector;

class C4Vector {
    public:
    // Member variables
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 0.0f;

    // Member functions
    C4Vector() = default;
    C4Vector(float x, float y, float z, float w)
        : x(x)
        , y(y)
        , z(z)
        , w(w) {};
    C4Vector(C3Vector v);
};

C4Vector operator*(const C4Vector& l, const C44Matrix& r);

#endif
