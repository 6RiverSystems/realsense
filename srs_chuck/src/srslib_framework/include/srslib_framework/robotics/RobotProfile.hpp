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

        goalReachedDistance(0.0),
        goalReachedAngle(0.0),

        maxAngularAcceleration(0.0),
        maxAngularVelocity(0.0),
        maxLinearAcceleration(0.0),
        maxLinearVelocity(0.0),
        maxLookAheadDistance(0.0),
        minInitialLinearVelocity(0.0),
        minLookAheadDistance(0.0),
        minLinearVelocity(0.0),

        ratioEmergency(0.0),
        ratioLookAheadDistance(0.0),
        ratioManual(0.0),

        travelAngularAcceleration(0.0),
        travelAngularVelocity(0.0),
        travelCurveZoneRadius(0.0),
        travelCurvingVelocity(0.0),
        travelLinearAcceleration(0.0),
        travelLinearVelocity(0.0),
        travelRotationVelocity(0.0),

        wheelDiameter(0.0),
        wheelDistance(0.0)
    {}

    virtual ~RobotProfile()
    {}

    double bodyWidth; // [m]
    double bodyDepth; // [m]

    double goalReachedDistance; // [m]
    double goalReachedAngle; // [rad]

    double maxAngularAcceleration; // [rad/s^2]
    double maxAngularVelocity; // [rad/s]
    double maxLinearAcceleration; // [m/s^2]
    double maxLinearVelocity; // [m/s]
    double maxLookAheadDistance; // [m]
    double minInitialLinearVelocity; // [m/s]
    double minLookAheadDistance; // [m]
    double minLinearVelocity; // [m/s]

    double ratioEmergency; // []
    double ratioLookAheadDistance; // []
    double ratioManual; // []

    double travelAngularAcceleration; // [rad/s^2]
    double travelAngularVelocity; // [rad/s]
    double travelCurveZoneRadius; // [m]
    double travelCurvingVelocity; // [m/s]
    double travelLinearAcceleration; // [m/s^2]
    double travelLinearVelocity; // [m/s]
    double travelRotationVelocity; // [rad/s]

    double wheelDiameter; // [m]
    double wheelDistance; // [m]
};

} // namespace srs

#endif // ROBOTPROFILE_HPP_
