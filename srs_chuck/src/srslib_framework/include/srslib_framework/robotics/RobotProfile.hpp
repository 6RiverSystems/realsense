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
        adaptiveLookAhead(true),

        bodyWidth(0.0),
        bodyDepth(0.0),

        emergencyRatioCrawl(0.0),
        emergencyKv(1.0),
        emergencyKw(1.0),

        goalReachedDistance(0.0),
        goalReachedAngle(0.0),

        manualRatioAngular(0.0),
        manualRatioLinear(0.0),
        manualKv(1.0),
        manualKw(1.0),
        maxAngularAcceleration(0.0),
        maxAngularVelocity(0.0),
        maxLinearAcceleration(0.0),
        maxLinearVelocity(0.0),
        maxLookAheadDistance(0.0),
        minAngularVelocity(0.0),
        minLinearVelocity(0.0),
        minLookAheadDistance(0.0),
        minPhysicalAngularVelocity(0.0),
        minPhysicalLinearVelocity(0.0),

        pathFollowKv(1.0),
        pathFollowKw(1.0),

        rotationKd(0.0),
        rotationKi(0.0),
        rotationKp(1.0),
        rotationKv(1.0),
        rotationKw(1.0),

        smallStraightDistance(0.0),

        travelAngularAcceleration(0.0),
        travelAngularVelocity(0.0),
        travelTurningZoneRadius(0.0),
        travelTurningVelocity(0.0),
        travelLinearAcceleration(0.0),
        travelLinearVelocity(0.0),
        travelRotationVelocity(0.0),

        wheelDiameter(0.0),
        wheelDistance(0.0),

        zeroLookAheadDistance(0.0)
    {}

    virtual ~RobotProfile()
    {}

    bool adaptiveLookAhead;

    double bodyWidth; // [m]
    double bodyDepth; // [m]

    double emergencyRatioCrawl; // []
    double emergencyKv; // []
    double emergencyKw; // []

    double goalReachedDistance; // [m]
    double goalReachedAngle; // [rad]

    double manualKv; // []
    double manualKw; // []
    double manualRatioAngular; // []
    double manualRatioLinear; // []
    double maxAngularAcceleration; // [rad/s^2]
    double maxAngularVelocity; // [rad/s]
    double maxLinearAcceleration; // [m/s^2]
    double maxLinearVelocity; // [m/s]
    double maxLookAheadDistance; // [m]
    double minAngularVelocity; // [rad/s]
    double minLinearVelocity; // [m/s]
    double minLookAheadDistance; // [m]
    double minPhysicalAngularVelocity; // [rad/s]
    double minPhysicalLinearVelocity; // [m/s]

    double pathFollowKv; // []
    double pathFollowKw; // []

    double rotationKd; // []
    double rotationKi; // []
    double rotationKp; // []
    double rotationKv; // []
    double rotationKw; // []

    double smallStraightDistance; // [m]

    double travelAngularAcceleration; // [rad/s^2]
    double travelAngularVelocity; // [rad/s]
    double travelTurningZoneRadius; // [m]
    double travelTurningVelocity; // [m/s]
    double travelLinearAcceleration; // [m/s^2]
    double travelLinearVelocity; // [m/s]
    double travelRotationVelocity; // [rad/s]

    double wheelDiameter; // [m]
    double wheelDistance; // [m]

    double zeroLookAheadDistance; // [m]
};

} // namespace srs

#endif // ROBOTPROFILE_HPP_
