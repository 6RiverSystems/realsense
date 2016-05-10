/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef CHUCK_HPP_
#define CHUCK_HPP_

#include <srslib_framework/math/Math.hpp>
#include <srslib_framework/robotics/robot/RobotProfile.hpp>

namespace srs {

struct Chuck : RobotProfile
{
    virtual ~Chuck()
    {}

    double bodyWidth() // [m]
    {
        return Math::inch2m<double>(24.375);
    }

    double bodyDepth() // [m]
    {
        return Math::inch2m<double>(38.474);
    }

    double linearAccelerationTravelMax() // [m/s^2]
    {
        return 1.0;
    }

    double linearVelocityTravelMax() // [m/s]
    {
        return 1.0;
    }

    virtual double wheelDiameter() // [m]
    {
        return Math::inch2m<double>(8);
    }

    virtual double wheelDistance() // [m]
    {
        return Math::inch2m<double>(20.915);
    }

//    MAX_OMEGA = deg2rad(120.0); % [rad/s]
//    TRAVEL_OMEGA = deg2rad(90.0); % [rad/s]
//
//    MAX_ALPHA = deg2rad(120.0); % [rad/s^2]
//    TRAVEL_ALPHA = deg2rad(90.0); % [rad/s^2]
};

} // namespace srs

#endif // CHUCK_HPP_
