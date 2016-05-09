/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROBOTPROFILE_HPP_
#define ROBOTPROFILE_HPP_

#include <srslib_framework/math/Math.hpp>

namespace srs {

struct RobotProfile
{
    virtual ~RobotProfile()
    {}

    virtual double bodyWidth() = 0; // [m]
    virtual double bodyDepth() = 0; // [m]

    virtual double linearAccelerationTravelMax() = 0; // [m/s^2]
    virtual double linearVelocityTravelMax() = 0; // [m/s]

    virtual double wheelDiameter() = 0; // [m]
    virtual double wheelDistance() = 0; // [m]
};

} // namespace srs

#endif // ROBOTPROFILE_HPP_
