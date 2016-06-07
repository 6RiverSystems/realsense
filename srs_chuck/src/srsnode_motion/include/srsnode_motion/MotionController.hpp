/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MOTIONCONTROLLER_HPP_
#define MOTIONCONTROLLER_HPP_

#include <vector>
using namespace std;

#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

#include <srslib_framework/robotics/lowlevel_controller/YoshizawaController.hpp>
#include <srslib_framework/robotics/lowlevel_controller/CMUController.hpp>

#include <srslib_framework/robotics/robot/RobotProfile.hpp>

namespace srs {

class MotionController
{
public:
    MotionController();

    ~MotionController()
    {}

    Velocity<> getExecutingCommand()
    {
        newCommandAvailable_ = false;
        return executingCommand_;
    }

    Pose<> getGoal()
    {
        return goal_;
    }

    bool isGoalReached()
    {
        return goalReached_;
    }

    bool isMoving()
    {
        return moving_;
    }

    bool newCommandAvailable()
    {
        return newCommandAvailable_;
    }

    void reset(Pose<> robotPose);
    void run(double dT, Pose<> robotPose);

    void setRobotPose(Pose<> robotPose)
    {
        currentRobotPose_ = robotPose;
    }

    void setRobot(RobotProfile robot);

    void setTrajectory(Trajectory<> trajectory);
    void stop(double stopDistance = 0);

private:
    // TODO: find a better place for this
    bool similarVelocities(const Velocity<>& lhv, const Velocity<>& rhv)
    {
        // The two velocities to be similar must:
        // - difference in linear velocity must be less than 0.01 m/s
        // - difference in angular velocity must be less than 0.1 deg/s
        return abs(lhv.linear - rhv.linear) < 0.01 && abs(lhv.angular - rhv.angular) < 0.002;
    }

    void updateProjectionIndex();

    Pose<> currentRobotPose_;

    Velocity<> executingCommand_;

    Pose<> goal_;
    bool goalReached_;

    CMUController lowLevelController_;

    bool moving_;

    bool newCommandAvailable_;

    int projectionIndex_;

    Pose<> referencePose_;
    int referenceIndex_;
    RobotProfile robot_;

    Trajectory<> trajectory_;
};

} // namespace srs

#endif  // MOTIONCONTROLLER_HPP_
