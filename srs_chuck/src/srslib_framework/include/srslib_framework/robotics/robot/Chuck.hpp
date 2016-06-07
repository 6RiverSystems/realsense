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
    Chuck() :
        RobotProfile()
    {
        bodyWidth = Math::inch2m<double>(24.375); // [m]
        bodyDepth = Math::inch2m<double>(38.474); // [m]
        goalReachedDistance = 0.1; // [m]
        lookAheadDistance = 1.1; // [m]
        maxAngularAcceleration = 2.6; // [rad/s^2]
        maxAngularVelocity = 9.7; // [rad/s]
        maxLinearAcceleration = 0.7; // [m/s^2]
        maxLinearVelocity = 2.6; // [m/s]
        travelAngularAcceleration = 0.5; // [rad/s^2]
        travelAngularVelocity = 2.0; // [rad/s]
        travelLinearAcceleration =  0.65; // [m/s^2]
        travelLinearVelocity = 1.0; // [m/s]
        wheelDiameter = Math::inch2m<double>(8); // [m]
        wheelDistance = Math::inch2m<double>(20.915); // [m]
    }

    virtual ~Chuck()
    {}
};

} // namespace srs

#endif // CHUCK_HPP_
