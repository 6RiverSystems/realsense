/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef CMUPATHFOLLOWER_HPP_
#define CMUPATHFOLLOWER_HPP_

#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/controller/BaseController.hpp>

namespace srs {

/**
 * J. M. Snider, "Automatic Steering Methods for Autonomous
 * Automobile Path Tracking", Robotics Institute, Carnegie Mellon
 * University, Pittsburgh, PA, USA, Tech. Report CMU-RI-TR-09-08, Feb. 2009
 */
class CMUPathFollower: public BaseController
{
public:
    CMUPathFollower() :
        BaseController(),
        dynamicLookAheadDistance_(0.5),
        maxLookAheadDistance_(0.5),
        minLookAheadDistance_(0.5),
        projectionIndex_(-1),
        ratioLookAheadDistance_(1.1),
        referencePose_(Pose<>()),
        referenceIndex_(-1),
        travelRotationVelocity_(0.1)
    {}

    ~CMUPathFollower()
    {}

    void reset();

    void setMaxLookAheadDistance(double lookAheadDistance)
    {
        maxLookAheadDistance_ = lookAheadDistance;
    }

    void setMinLookAheadDistance(double lookAheadDistance)
    {
        minLookAheadDistance_ = lookAheadDistance;
    }

    void setRatioLookAheadDistance(double ratioLookAheadDistance)
    {
        ratioLookAheadDistance_ = ratioLookAheadDistance;
    }

    void setTrajectory(Trajectory<> trajectory, Pose<> robotPose);

    void setTravelRotationVelocity(double value)
    {
        travelRotationVelocity_ = value;
    }

    void stepController(Pose<> currentPose, Odometry<> currentOdometry);

private:
    void updateLookAheadDistance();
    void updateProjectionIndex(Pose<> robotPose);

    Trajectory<> currentTrajectory_;

    double dynamicLookAheadDistance_;

    double maxLookAheadDistance_;
    double minLookAheadDistance_;

    int projectionIndex_;

    double ratioLookAheadDistance_;
    Pose<> referencePose_;
    int referenceIndex_;

    double travelRotationVelocity_;
};

} // namespace srs

#endif // CMUPATHFOLLOWER_HPP_
