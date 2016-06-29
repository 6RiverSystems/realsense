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

        emergencyKv = 1.0; // []
        emergencyKw = 1.0; // []
        emergencyMaxAngularVelocity = 0.15; // [rad/s]
        emergencyMaxLinearVelocity = 0.080; // [m/s]

        manualKv = 1.0; // []
        manualKw = 1.0; // []
        manualMaxAngularVelocity = 0.40; // [rad/s]
        manualMaxLinearVelocity = 0.400; // [m/s]

        physicalMaxAngularAcceleration = AngleMath::deg2rad<double>(150.0); // 2.618 [rad/s^2]
        physicalMaxAngularVelocity = AngleMath::deg2rad<double>(555.0); // 9.687 [rad/s]
        physicalMaxLinearAcceleration = 0.700; // [m/s^2]
        physicalMaxLinearVelocity = 2.600; // [m/s]
        physicalMinAngularVelocity = AngleMath::deg2rad(0.05); // 0.0008 [rad/s]
        physicalMinLinearVelocity = 0.005; // [m/s]

        pathFollowAdaptiveLookAhead = false;
        pathFollowGoalReachedDistance = 0.025; // [m]
        pathFollowKv = 1.0; // []
        pathFollowKw = 1.0; // []
        pathFollowLandingDepth = 1.0; // [m]
        pathFollowLandingWidth = 1.0; // [m]
        pathFollowLinearAcceleration =  0.250; // [m/s^2]
        pathFollowMaxAngularVelocity = AngleMath::deg2rad<double>(40); // 0.7000 [rad/s]
        pathFollowMaxLinearVelocity = 1.000; // [m]
        pathFollowMaxLookAheadDistance = 1.100; // [m]
        pathFollowMinLinearVelocity = 0.010; // [m/s]
        pathFollowMinLookAheadDistance = 0.800; // [m]
        pathFollowSmallStraightDistance = 3.0; // [m]
        pathFollowTurningVelocity = 0.300; // [m/s]
        pathFollowTurningZoneRadius = 0.500; // [m]
        pathFollowZeroLookAheadDistance = 0.800; // [m]

        rotationGoalReachedAngle = AngleMath::deg2rad<double>(1.0); // 0.0175 [rad]
        rotationKd = 0.0; // []
        rotationKi = 0.0; // []
        rotationKp = 2.25; // []
        rotationKv = 1.0; // []
        rotationKw = 1.0; // []
        rotationMinAngularVelocity = AngleMath::deg2rad(0.05); // 0.0008 [rad/s]
        rotationRotationVelocity = AngleMath::deg2rad(40.0); // 0.7 [rad/s]

        stopMinLinearVelocity = 0.010; // [m/s]
        stopNormalDeceleration = 0.650; // [m/s^2]

        wheelDiameter = MeasurementMath::inch2m<double>(8.000); // 0.203 [m]
        wheelDistance = MeasurementMath::inch2m<double>(20.915); // 0.531 [m]
    }

    virtual ~Chuck()
    {}
};

} // namespace srs

#endif // CHUCK_HPP_
