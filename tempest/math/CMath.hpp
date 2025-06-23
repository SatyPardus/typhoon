#ifndef TEMPEST_MATH_C_MATH_HPP
#define TEMPEST_MATH_C_MATH_HPP

#include <cmath>
#include <cstdint>
#include <storm/Error.hpp>

class CMath {
    public:
    // Static variables
    static constexpr float PI = 3.1415927f;
    static constexpr float TWO_PI = 6.2831855f;
    static constexpr float OO_TWO_PI = 1.0f / TWO_PI;

    // Static functions
    static float acos(float x) {
        return ::acosf(x);
    }

    static float cos(float x) {
        return ::cosf(x);
    }

    static float sin(float x) {
        return ::sinf(x);
    }

    static float fabs(float x) {
        return ::fabs(x);
    }

    static bool fequalz(float a, float b, float z) {
        return z > CMath::fabs(a - b);
    }

    static bool fequal(float a, float b) {
        return CMath::fequalz(a, b, 0.00000023841858f);
    }

    static int32_t fint(float n) {
        return static_cast<int32_t>(n);
    }

    static int32_t fint_n(float n) {
        return n <= 0.0f ? static_cast<int32_t>(n - 0.5f) : static_cast<int32_t>(n + 0.5f);
    }

    static uint32_t fuint(float n) {
        return static_cast<uint32_t>(n);
    }

    static uint32_t fuint_n(float n) {
        return static_cast<uint32_t>(n + 0.5f);
    }

    static uint32_t fuint_pi(float n) {
        return static_cast<uint32_t>(n + 0.99994999);
    }

    static float sqrt(float x) {
        STORM_ASSERT(x >= 0.0f);
        return ::sqrt(x);
    }

    static void normalize(float& x, float& y) {
        float m = x * x + y * y;
        STORM_ASSERT(m >= 0.0f);
        m = 1.0f / CMath::sqrt(m);
        x *= m;
        y *= m;
    }

    static void normalize(float& x, float& y, float& z) {
        float m = x * x + y * y + z * z;
        STORM_ASSERT(m >= 0.0f);
        m = 1.0f / CMath::sqrt(m);
        x *= m;
        y *= m;
        z *= m;
    }
};

#endif
