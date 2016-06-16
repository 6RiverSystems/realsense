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
    template<typename TYPE = double>
    inline static bool equal(const Velocity<TYPE>& lhv, const Velocity<TYPE>& rhv)
    {
        // The two velocities to be similar must:
        //
        // - difference in linear velocity must be less than 0.001 m/s
        // - difference in angular velocity must be less than 0.05 deg/s
        //
        return BasicMath::equal(lhv.linear, rhv.linear, 0.001) &&
            AngleMath::equalRad(lhv.angular, rhv.angular, 0.001);
    }

};

} // namespace srs

#endif // VELOCITYMATH_HPP_
