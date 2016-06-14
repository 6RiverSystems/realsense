/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef CHUCK_HPP_
#define CHUCK_HPP_

#include <srslib_framework/math/MeasurementMath.hpp>
#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/robotics/RobotProfile.hpp>

namespace srs {

struct Chuck : RobotProfile
{
    Chuck() :
        RobotProfile()
    {
        bodyWidth = MeasurementMath::inch2m<double>(24.375); // 0.619 [m]
        bodyDepth = MeasurementMath::inch2m<double>(38.474); // 0.977 [m]

        goalReachedDistance = 0.100; // [m]
        goalReachedAngle = AngleMath::deg2rad<double>(0.5); // 0.009 [rad]
        maxAngularAcceleration = AngleMath::deg2rad<double>(150.0); // 2.618 [rad/s^2]
        maxAngularVelocity = AngleMath::deg2rad<double>(555.0); // 9.687 [rad/s]
        maxLinearAcceleration = 0.700; // [m/s^2]
        maxLinearVelocity = 2.600; // [m/s]
        maxLookAheadDistance = 1.500; // [m]
        minLookAheadDistance = 0.500; // [m]

        ratioLookAheadDistance = 1.1; // []

        travelAngularAcceleration = AngleMath::deg2rad<double>(28.0); // 0.489 [rad/s^2]
        travelAngularVelocity = AngleMath::deg2rad<double>(115); // 2.007 [rad/s]
        travelCurveZoneRadius = 0.500; // [m]
        travelCurvingVelocity = 0.300; // [m/s]
        travelLinearAcceleration =  0.650; // [m/s^2]
        travelLinearVelocity = 1.000; // [m/s]
        travelRotationVelocity = AngleMath::deg2rad(20.0); // 0.349 [rad/s]

        wheelDiameter = MeasurementMath::inch2m<double>(8.000); // 0.203 [m]
        wheelDistance = MeasurementMath::inch2m<double>(20.915); // 0.531 [m]
    }

    virtual ~Chuck()
    {}
};

} // namespace srs

#endif // CHUCK_HPP_
