/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef BASECONTROLLER_HPP_
#define BASECONTROLLER_HPP_

#include <ros/ros.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/VelocityMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Odometry.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/robotics/RobotProfile.hpp>

namespace srs {

class BaseController
{
public:
    BaseController(string name) :
        canceled_(false),
        executingCommand_(Velocity<>()),
        goal_(Pose<>()),
        goalReached_(false),
        Kv_(1.0),
        Kw_(1.0),
        isRobotMoving_(false),
        name_(name),
        robot_()
    {}

    virtual ~BaseController()
    {}

    void cancel()
    {
        canceled_ = true;
        goalReached_ = true;
    }

    Velocity<> getExecutingCommand()
    {
        return executingCommand_;
    }

    Pose<> getGoal() const
    {
        return goal_;
    }

    string getName() const
    {
        return name_;
    }

    bool isCanceled() const
    {
        return canceled_;
    }

    bool isGoalReached() const
    {
        return goalReached_;
    }

    bool isRobotMoving() const
    {
        return isRobotMoving_;
    }

    virtual void reset()
    {
        canceled_ = false;

        goal_ = Pose<>();
        goalReached_ = false;

        isRobotMoving_ = false;

        executingCommand_ = Velocity<>();
    }

    void setGoal(Pose<> goal)
    {
        goal_ = goal;
    }

    void setRobotProfile(RobotProfile robot)
    {
        robot_ = robot;
    }

    void setVelocityGains(double Kv, double Kw)
    {
        Kv_ = Kv;
        Kw_ = Kw;
    }

    void step(double dT, Pose<> currentPose, Odometry<> currentOdometry)
    {
        // Perform some basic operations before calling
        // the specific controller step
        isRobotMoving_ = !VelocityMath::equal<double>(currentOdometry.velocity, Velocity<>());

        // Call the controller
        stepController(dT, currentPose, currentOdometry);
    }

protected:
    void executeCommand(Velocity<> command)
    {
        ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "BaseController", "Executing command: " << command);
        executingCommand_ = command;
    }

    virtual void stepController(double dT, Pose<> currentPose, Odometry<> currentOdometry) = 0;

    bool canceled_;

    Velocity<> executingCommand_;

    Pose<> goal_;
    bool goalReached_;

    double Kv_;
    double Kw_;

    bool isRobotMoving_;

    string name_;

    RobotProfile robot_;
};

} // namespace srs

#endif // BASECONTROLLER_HPP_
