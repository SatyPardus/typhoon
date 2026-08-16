#include "tempest/rect/CRect.hpp"
#include "tempest/Vector.hpp"

CRect CRect::Intersection(const CRect& l, const CRect& r) {
    CRect i;

    i.maxX = r.maxX <= l.maxX ? r.maxX : l.maxX;
    i.maxY = r.maxY <= l.maxY ? r.maxY : l.maxY;
    i.minX = r.minX >= l.minX ? r.minX : l.minX;
    i.minY = r.minY >= l.minY ? r.minY : l.minY;

    return i;
}

bool CRect::Intersects(const CRect& r) const {
    return this->maxX >= r.minX && this->maxY >= r.minY && this->minX <= r.maxX && this->minY <= r.maxY;
}

bool CRect::operator==(const CRect& rect) {
    return this->minX == rect.minX && this->minY == rect.minY && this->maxX == rect.maxX && this->maxY == rect.maxY;
}

bool CRect::IsPointInside(const C2Vector& pt) {
    return this->minX <= pt.x && this->maxX >= pt.x && this->minY <= pt.y && this->maxY >= pt.y;
}

bool CRect::Sub4826D0() const {
    return this->maxY < 0.0f || this->minY > 1.0f || this->maxX < 0.0f || this->minX > 1.0f;
}

// OFFSET: 0x78F2F0
CRect CRect::Union(const CRect& r) {
    float maxX = (r.maxX >= this->maxX) ? r.maxX : this->maxX;
    float maxY = (r.maxY >= this->maxY) ? r.maxY : this->maxY;
    float minX = (r.minX <= this->minX) ? r.minX : this->minX;
    float minY = (r.minY <= this->minY) ? r.minY : this->minY;

    CRect result;
    result.minY = minY;
    result.minX = minX;
    result.maxY = maxY;
    result.maxX = maxX;

    return result;
}

// OFFSET: 0x7A6DD0
CRect& CRect::operator/=(const CRect& a) {
    this->minY /= a.minY;
    this->minX /= a.minX;
    this->maxY /= a.maxY;
    this->maxX /= a.maxX;
    return *this;
}

// OFFSET: 0x7A6DA0
CRect operator+(const CRect& l, const CRect& r) {
    float minY = l.minY + r.minY;
    float minX = l.minX + r.minX;
    float maxY = l.maxY + r.maxY;
    float maxX = l.maxX + r.maxX;

    return { minY, minX, maxY, maxX };
}
