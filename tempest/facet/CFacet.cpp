#include "tempest/facet/CFacet.hpp"

// OFFSET: 0x5FED80
CFacet::CFacet(float a2) {
    this->plane.n.x = 0.0;
    this->plane.n.y = 0.0;
    this->plane.n.z = 1.0;
    this->plane.d = 0.0;
    this->v[0].x = 0.0;
    this->v[0].y = 0.0;
    this->v[0].z = 0.0;
    this->v[1].x = 0.0;
    this->v[1].y = 0.0;
    this->v[1].z = 0.0;
    this->v[2].x = 0.0;
    this->v[2].y = 0.0;
    this->v[2].z = 0.0;
    this->Set(a2);
}

// OFFSET: 0x983940
CFacet::CFacet(C3Vector& v1, C3Vector& v2, C3Vector& v3) {
    this->v[0] = v1;
    this->v[1] = v2;
    this->v[2] = v3;
    this->plane.From3Pos(v1, v2, v3);
}

// OFFSET: 0x9838D0
void CFacet::Set(float a2) {
    this->v[0].x = a2;
    this->v[0].y = a2;
    this->v[0].z = a2;
    this->v[1].x = a2;
    this->v[1].y = a2;
    this->v[1].z = a2;
    this->v[2].x = a2;
    this->v[2].y = a2;
    this->v[2].z = a2;
    this->plane.n.x = 0.0;
    this->plane.n.y = 0.0;
    this->plane.n.z = 1.0;
    this->plane.d = (this->v[0].y + this->v[0].x) * -0.0f - this->v[0].z;
}
