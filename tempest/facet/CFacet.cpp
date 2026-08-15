#include "tempest/facet/CFacet.hpp"

CFacet::CFacet(C3Vector& v1, C3Vector& v2, C3Vector& v3) {
    this->v[0] = v1;
    this->v[1] = v2;
    this->v[2] = v3;
    this->plane.From3Pos(v1, v2, v3);
}
