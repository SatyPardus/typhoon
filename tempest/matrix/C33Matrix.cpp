#include "tempest/matrix/C33Matrix.hpp"
#include "tempest/math/CMath.hpp"

#include <storm/Error.hpp>


float C33Matrix::Det(float a, float b, float c, float d) {
    return (a * d) - (b * c);
}

C33Matrix C33Matrix::Rotation(float angle) {
    float cosa = cos(angle);
    float sina = sin(angle);

    C33Matrix result;
    result.a0 = cosa;
    result.a1 = sina;
    result.a2 = 0.0;

    result.b0 = -sina;
    result.b1 = cosa;
    result.b2 = 0.0;

    result.c0 = 0.0;
    result.c1 = 0.0;
    result.c2 = 1.0;
    return result;
}

C33Matrix C33Matrix::Rotation(float angle, const C3Vector& axis, bool unit) {
    C3Vector axis_ = axis;
    if (!unit) {
        axis_.Normalize();
    }
    STORM_ASSERT(axis_.IsUnit());

    float sina = CMath::sin(angle);
    float cosa = CMath::cos(angle);

    float xs = axis_.x * sina;
    float ys = axis_.y * sina;
    float zs = axis_.z * sina;

    float one_c = 1.0f - cosa;

    C33Matrix result;

    result.a0 = axis_.x * axis_.x * one_c + cosa;
    result.a1 = one_c * axis_.y * axis_.x + zs;
    result.a2 = one_c * axis_.z * axis_.x - ys;

    result.b0 = one_c * axis_.y * axis_.x - zs;
    result.b1 = axis_.y * axis_.y * one_c + cosa;
    result.b2 = one_c * axis_.z * axis_.y + xs;

    result.c0 = one_c * axis_.z * axis_.x + ys;
    result.c1 = one_c * axis_.z * axis_.y - xs;
    result.c2 = axis_.z * axis_.z * one_c + cosa;

    return result;
}
