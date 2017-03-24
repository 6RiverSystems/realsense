/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef CMUPATHFOLLOWER_HPP_
#define CMUPATHFOLLOWER_HPP_

#include <srslib_framework/planning/pathplanning/trajectory/Trajectory.hpp>
#include <srslib_framework/robotics/controller/BaseController.hpp>

namespace srs {

/**
 * Based on the technical report:
 *
 * J. M. Snider, "Automatic Steering Methods for Autonomous
 * Automobile Path Tracking", Robotics Institute, Carnegie Mellon
 * University, Pittsburgh, PA, USA, Tech. Report CMU-RI-TR-09-08, Feb. 2009
 */
class CMUPathController: public BaseController
{
public:
    CMUPathController() :
        BaseController("CMU PATH CONTROLLER"),
        currentVelocity_(0.0),
        lookAheadDistance_(1.0),
        projectionIndex_(-1),
        referencePose_(Pose<>::ZERO),
        velocityChange_(0.0),
        velocityChangePose_(Pose<>::ZERO),
        velocityCurrentMax_(0.0)
    {}

    ~CMUPathController()
    {}

    double getLookAheadDistance() const
    {
        return lookAheadDistance_;
    }

    void reset();

    void setRobotProfile(RobotProfile robot)
    {
        BaseController::setRobotProfile(robot, robot.pathFollowKv, robot.pathFollowKw);
    }

    void setTrajectory(Pose<> robotPose, Trajectory<> trajectory, Pose<> goal);

protected:
    void calculateLanding(Pose<> goal);
    void stepController(double dT, Pose<> currentPose, Odometry<> currentOdometry);

private:
    int findVelocityChange(double fromVelocity, int fromIndex);

    double getMaxVelocity(int position);
    bool goalReached(Pose<> currentPose);

    void updateParameters(Pose<> currentPose);

    Trajectory<> currentTrajectory_;
    double currentVelocity_;

    double lookAheadDistance_;

    int projectionIndex_;

    Pose<> referencePose_;

    double velocityChange_;
    Pose<> velocityChangePose_;
    double velocityCurrentMax_;
};

} // namespace srs

#endif // CMUPATHFOLLOWER_HPP_
