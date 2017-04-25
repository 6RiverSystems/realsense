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
    /**
     * @brief Compare two angles.
     *
     * @tparam TYPE Basic type of the operation
     * @param lhs Left-hand side angle [radians]
     * @param rhs Right-hand side angle [radians]
     * @param threshold Threshold of the comparison [radians]
     *
     * @return true if the difference between the two angles is less than the specified threshold
     */
    template<typename TYPE = double>
    inline static bool equalRad(TYPE lhs, TYPE rhs, TYPE threshold = numeric_limits<TYPE>::epsilon())
    {
        return abs(AngleMath::normalizeRad<TYPE>(lhs - rhs)) < threshold;
    }

    /**
     * @brief Convert degrees to radians.
     *
     * @tparam TYPE Basic type of the operation
     * @param deg Angle to convert [degrees]
     *
     * @return converted angle [radians]
     */
    template<typename TYPE = double>
    constexpr inline static TYPE deg2Rad(TYPE deg)
    {
        return deg * TYPE(M_PI) / TYPE(180);
    }

    /**
     * @brief Normalize the specified angle to a [0, 360) range.
     *
     * @tparam TYPE Basic type of the operation
     * @param deg Angle to convert [degrees]
     *
     * @return converted angle [degrees]
     */
    template<typename TYPE = double>
    inline static TYPE normalizeDeg(TYPE deg)
    {
        return deg < TYPE(0) ?
            TYPE(360) - fmod(std::abs(deg), TYPE(360)) :
            fmod(deg, TYPE(360));
    }

    template<int>
    inline static int normalizeDeg(int deg)
    {
        return deg < 0 ?
            360 - (static_cast<int>(std::abs<int>(deg)) % 360) :
            deg % 360;
    }

    /**
     * @brief Normalize the specified angle to a [-pi, pi) range.
     *
     * @tparam TYPE Basic type of the operation
     * @param deg Angle to normalize [degrees]
     *
     * @return normalized angle to a [-pi, pi) range [radians]
     */
    template<typename TYPE = double>
    inline static TYPE normalizeDeg2Rad(TYPE deg)
    {
        return AngleMath::normalizeRad<TYPE>(AngleMath::deg2Rad<TYPE>(deg));
    }

    /**
     * @brief Normalize the specified angle to a [-pi, pi) range.
     *
     * @tparam TYPE Basic type of the operation
     * @param rad Angle to normalize [radians]
     *
     * @return normalized angle to a [-pi, pi) range
     */
    template<typename TYPE = double>
    constexpr inline static TYPE normalizeRad(TYPE rad)
    {
        return rad - 2 * M_PI * floor((rad + M_PI) / (2 * M_PI));
    }

    /**
     * @brief Normalize the specified angle to a [0, 360) range.
     *
     * @tparam TYPE Basic type of the operation
     * @param rad Angle to normalize [radians]
     *
     * @return normalized angle to a [0, 360) range [degrees]
     */
    template<typename TYPE = double>
    inline static TYPE normalizeRad2Deg(TYPE rad)
    {
        return AngleMath::normalizeDeg<TYPE>(AngleMath::rad2Deg<TYPE>(rad));
    }

    /**
     * @brief Normalize the specified angle to a multiple of 90 deg.
     *
     * @tparam TYPE Basic type of the operation
     * @param rad Angle to normalize [radians]
     *
     * @return angle normalized to multiples of 90 deg [degrees]
     */
    inline static double normalizeRad2Deg90(double rad)
    {
        double angle = AngleMath::rad2Deg<double>(rad);
        double ratio = angle / 90.0;
        double upper = ceil(ratio) * 90.0;
        double lower = floor(ratio) * 90.0;

        angle = (upper - angle) > (angle - lower) ? lower : upper;
        return AngleMath::normalizeDeg<double>(angle);
    }

    inline static double normalizeRad2Rad90(double rad)
    {
        return AngleMath::deg2Rad<double>(AngleMath::normalizeRad2Deg90(rad));
    }

    /**
     * @brief Convert radians to degrees.
     *
     * @tparam TYPE Basic type of the operation
     * @param rad Angle to convert [radians]
     *
     * @return converted angle [degrees]
     */
    template<typename TYPE = double>
    constexpr inline static TYPE rad2Deg(TYPE rad)
    {
        return rad * TYPE(180) / TYPE(M_PI);
    }
};

} // namespace srs

#endif // ANGLEMATH_HPP_
