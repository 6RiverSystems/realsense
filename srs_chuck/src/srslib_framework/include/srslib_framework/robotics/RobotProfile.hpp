/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROBOTPROFILE_HPP_
#define ROBOTPROFILE_HPP_

namespace srs {

struct RobotProfile
{
    RobotProfile() :
        bodyWidth(0.0),
        bodyDepth(0.0),

        emergencyKv(1.0),
        emergencyKw(1.0),
        emergencyMaxAngularVelocity(0.0),
        emergencyMaxLinearVelocity(0.0),

        manualKv(1.0),
        manualKw(1.0),
        manualMaxAngularVelocity(0.0),
        manualMaxLinearVelocity(0.0),

        physicalMaxAngularAcceleration(0.0),
        physicalMaxAngularVelocity(0.0),
        physicalMaxLinearAcceleration(0.0),
        physicalMaxLinearVelocity(0.0),
        physicalMinAngularVelocity(0.0),
        physicalMinLinearVelocity(0.0),

        pathFollowAdaptiveLookAhead(true),
        pathFollowGoalReachedDistance(0.0),
        pathFollowKv(1.0),
        pathFollowKw(1.0),
        pathFollowLandingDepth(0.0),
        pathFollowLandingWidth(0.0),
        pathFollowLinearAcceleration(0.0),
        pathFollowMaxAngularVelocity(0.0),
        pathFollowMaxLinearVelocity(0.0),
        pathFollowMaxLookAheadDistance(0.0),
        pathFollowMinLinearVelocity(0.0),
        pathFollowMinLookAheadDistance(0.0),
        pathFollowSmallStraightDistance(0.0),
        pathFollowTurningVelocity(0.0),
        pathFollowTurningZoneRadius(0.0),
        pathFollowZeroLookAheadDistance(0.0),

        rotationGoalReachedAngle(0.0),
        rotationKd(0.0),
        rotationKi(0.0),
        rotationKp(1.0),
        rotationKv(1.0),
        rotationKw(1.0),
        rotationMinAngularVelocity(0.0),
        rotationRotationVelocity(0.0),

        stopMinLinearVelocity(0.0),
        stopNormalDeceleration(0.0),

        wheelDiameter(0.0),
        wheelDistance(0.0)
    {}

    virtual ~RobotProfile()
    {}

    double bodyWidth; // [m]
    double bodyDepth; // [m]

    double emergencyKv; // []
    double emergencyKw; // []
    double emergencyMaxAngularVelocity; // [rad/s]
    double emergencyMaxLinearVelocity; // [m/s]

    double manualKv; // []
    double manualKw; // []
    double manualMaxAngularVelocity; // [rad/s]
    double manualMaxLinearVelocity; // [m/s]

    double physicalMaxAngularAcceleration; // [rad/s^2]
    double physicalMaxAngularVelocity; // [rad/s]
    double physicalMaxLinearAcceleration; // [m/s^2]
    double physicalMaxLinearVelocity; // [m/s]
    double physicalMinAngularVelocity; // [rad/s]
    double physicalMinLinearVelocity; // [m/s]

    bool pathFollowAdaptiveLookAhead;
    double pathFollowGoalReachedDistance; // [m]
    double pathFollowKv; // []
    double pathFollowKw; // []
    double pathFollowLandingDepth; // [m]
    double pathFollowLandingWidth; // [m]
    double pathFollowLinearAcceleration; // [m/s^2]
    double pathFollowMaxAngularVelocity; // [rad/s]
    double pathFollowMaxLinearVelocity; // [m/s]
    double pathFollowMaxLookAheadDistance; // [m]
    double pathFollowMinLinearVelocity; // [m/s]
    double pathFollowMinLookAheadDistance; // [m]
    double pathFollowSmallStraightDistance; // [m]
    double pathFollowTurningVelocity; // [m/s]
    double pathFollowTurningZoneRadius; // [m]
    double pathFollowZeroLookAheadDistance; // [m]

    double rotationGoalReachedAngle; // [rad]
    double rotationKd; // []
    double rotationKi; // []
    double rotationKp; // []
    double rotationKv; // []
    double rotationKw; // []
    double rotationMinAngularVelocity; // [rad/s]
    double rotationRotationVelocity; // [rad/s]

    double stopMinLinearVelocity; // [m/s]
    double stopNormalDeceleration; // [m/s^2]

    double wheelDiameter; // [m]
    double wheelDistance; // [m]
};

} // namespace srs

#endif // ROBOTPROFILE_HPP_
