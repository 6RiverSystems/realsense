/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ANGLEMATH_HPP_
#define ANGLEMATH_HPP_

#include <cmath>
#include <limits>
using namespace std;

namespace srs {

struct AngleMath
{
    template<typename TYPE = double>
    constexpr inline static TYPE deg2rad(TYPE deg)
    {
        return deg * TYPE(M_PI) / TYPE(180);
    }

    template<typename TYPE = int>
    inline static TYPE normalizeAngleDeg(TYPE deg)
    {
        return deg < 0 ? 360 - (abs(deg) % 360) : deg % 360;
    }

    inline static unsigned int normalizeRad2deg90(double rad)
    {
        double angle = AngleMath::rad2deg<double>(rad);
        double ratio = angle / 90.0;
        double upper = ceil(ratio) * 90.0;
        double lower = floor(ratio) * 90.0;

        angle = (upper - angle) > (angle - lower) ? lower : upper;
        return AngleMath::normalizeAngleDeg<int>(static_cast<int>(angle));
    }

    /**
     * @brief Normalize the specified angle [rad] to a [-pi, pi) range.
     *
     * @tparam TYPE Basic type of the operation
     * @param rad Angle to normalize
     *
     * @return normalized angle to a [-pi, pi) range
     */
    template<typename TYPE = double>
    constexpr inline static TYPE normalizeAngleRad(TYPE rad)
    {
        return rad - 2 * M_PI * floor((rad + M_PI) / (2 * M_PI));
    }

    template<typename TYPE = double>
    constexpr inline static TYPE rad2deg(TYPE rad)
    {
        return rad * TYPE(180) / TYPE(M_PI);
    }
};

} // namespace srs

#endif // ANGLEMATH_HPP_