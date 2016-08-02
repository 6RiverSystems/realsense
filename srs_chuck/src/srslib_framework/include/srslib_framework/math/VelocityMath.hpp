/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef VELOCITYMATH_HPP_
#define VELOCITYMATH_HPP_

#include <cmath>
#include <limits>
using namespace std;

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

namespace srs {

struct VelocityMath
{
    /**
     * @brief Return TRUE if the velocities are similar. In order to consider them
     * similar:
     *
     * - The difference between linear velocities must be less than 0.001 m/s
     * - The difference between angular velocities must be less than 0.005 deg/s
     *
     * @param lhv Left hand velocity
     * @param rhv Right-hand velocity
     *
     * @return TRUE if the two velocities are similar
     */
    template<typename TYPE = double>
    inline static bool equal(const Velocity<TYPE>& lhv, const Velocity<TYPE>& rhv)
    {
        return BasicMath::equal(lhv.linear, rhv.linear, 0.001) &&
            AngleMath::equalRad(lhv.angular, rhv.angular, 0.001);
    }
};

} // namespace srs

#endif // VELOCITYMATH_HPP_
