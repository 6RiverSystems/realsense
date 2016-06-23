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
    static const Velocity<> ZERO_VELOCITY;
    static const Pose<> ZERO_POSE;

    BaseController(string name) :
        canceled_(false),
        executingCommand_(ZERO_VELOCITY),
        goal_(ZERO_POSE),
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

    virtual void reset();

    void setGoal(Pose<> goal)
    {
        goal_ = goal;
    }

    void step(double dT, Pose<> currentPose, Odometry<> currentOdometry);
    virtual void setRobotProfile(RobotProfile robot) = 0;

protected:
    void executeCommand(Velocity<> command);
    void executeCommand(double linear, double angular);

    virtual void stepController(double dT, Pose<> currentPose, Odometry<> currentOdometry) = 0;

    void setRobotProfile(RobotProfile robot, double Kv, double Kw)
    {
        robot_ = robot;

        Kv_ = Kv;
        Kw_ = Kw;
    }

    bool canceled_;

    Velocity<> executingCommand_;

    Pose<> goal_;
    bool goalReached_;

    bool isRobotMoving_;

    double Kv_;
    double Kw_;

    string name_;

    RobotProfile robot_;
};

} // namespace srs

#endif // BASECONTROLLER_HPP_
