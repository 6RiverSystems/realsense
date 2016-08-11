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
    inline static bool equalRad(TYPE a, TYPE b, TYPE threshold = numeric_limits<TYPE>::epsilon())
    {
        return abs(AngleMath::normalizeAngleRad<TYPE>(a - b)) < threshold;
    }

    template<typename TYPE = double>
    constexpr inline static TYPE deg2rad(TYPE deg)
    {
        return deg * TYPE(M_PI) / TYPE(180);
    }

    template<typename TYPE = int>
    inline static TYPE normalizeAngleDeg(TYPE deg)
    {
        while (deg < TYPE(0))
        {
            deg += TYPE(360);
        }

        while (deg > TYPE(360))
        {
            deg -= TYPE(360);
        }

        return deg;
    }

    template<typename TYPE = double>
    inline static TYPE normalizeRad2deg(TYPE rad)
    {
        TYPE angle = AngleMath::rad2deg<TYPE>(rad);

        return AngleMath::normalizeAngleDeg<TYPE>(static_cast<TYPE>(angle));
    }

    inline static int normalizeRad2Deg90(double rad)
    {
        double angle = AngleMath::rad2deg<double>(rad);
        double ratio = angle / 90.0;
        double upper = ceil(ratio) * 90.0;
        double lower = floor(ratio) * 90.0;

        angle = (upper - angle) > (angle - lower) ? lower : upper;
        return AngleMath::normalizeAngleDeg<int>(static_cast<int>(angle));
    }

    inline static double normalizeRad2Rad90(double rad)
    {
        return AngleMath::deg2rad<double>(AngleMath::normalizeRad2Deg90(rad));
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
