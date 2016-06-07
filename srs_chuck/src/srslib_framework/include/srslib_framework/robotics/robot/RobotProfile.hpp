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
    RobotProfile() :
        bodyWidth(),
        bodyDepth(),
        goalReachedDistance(),
        lookAheadDistance(),

        maxAngularAcceleration(),
        maxAngularVelocity(),
        maxLinearAcceleration(),
        maxLinearVelocity(),

        travelAngularAcceleration(),
        travelAngularVelocity(),
        travelLinearAcceleration(),
        travelLinearVelocity(),

        wheelDiameter(),
        wheelDistance()
    {}

    virtual ~RobotProfile()
    {}

    double bodyWidth; // [m]
    double bodyDepth; // [m]

    double goalReachedDistance; // [m]

    double lookAheadDistance; // [m]

    double maxAngularAcceleration; // [rad/s^2]
    double maxAngularVelocity; // [rad/s]
    double maxLinearAcceleration; // [m/s^2]
    double maxLinearVelocity; // [m/s]

    double travelAngularAcceleration; // [rad/s^2]
    double travelAngularVelocity; // [rad/s]
    double travelLinearAcceleration; // [m/s^2]
    double travelLinearVelocity; // [m/s]

    double wheelDiameter; // [m]
    double wheelDistance; // [m]
};

} // namespace srs

#endif // ROBOTPROFILE_HPP_
