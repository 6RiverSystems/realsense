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
class CMUPathController: public BaseController
{
public:
    CMUPathController() :
        BaseController(),
        dynamicLookAheadDistance_(0.5),
        projectionIndex_(-1),
        referencePose_(Pose<>()),
        referenceIndex_(-1)
    {}

    ~CMUPathController()
    {}

    void reset();

    void setTrajectory(Trajectory<> trajectory, Pose<> robotPose);

protected:
    void stepController(Pose<> currentPose, Odometry<> currentOdometry);

private:
    void updateLookAheadDistance();
    void updateProjectionIndex(Pose<> robotPose);

    Trajectory<> currentTrajectory_;

    double dynamicLookAheadDistance_;

    int projectionIndex_;

    Pose<> referencePose_;
    int referenceIndex_;
};

} // namespace srs

#endif // CMUPATHFOLLOWER_HPP_
