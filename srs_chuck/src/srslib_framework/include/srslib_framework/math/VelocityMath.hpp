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

#include <srslib_framework/robotics/Velocity.hpp>

namespace srs {

struct VelocityMath
{
    template<typename TYPE = double>
    inline static bool equal(const Velocity<TYPE>& lhv, const Velocity<TYPE>& rhv)
    {
        // The two velocities to be similar must:
        // - difference in linear velocity must be less than 0.01 m/s
        // - difference in angular velocity must be less than 0.1 deg/s

        // TODO: Move the constants to constexpr
        return abs(lhv.linear - rhv.linear) < 0.01 && abs(lhv.angular - rhv.angular) < 0.002;
    }

};

} // namespace srs

#endif // VELOCITYMATH_HPP_
