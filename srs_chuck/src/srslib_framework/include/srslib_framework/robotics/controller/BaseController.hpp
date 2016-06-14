/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef BASECONTROLLER_HPP_
#define BASECONTROLLER_HPP_

#include <ros/ros.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Odometry.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/robotics/RobotProfile.hpp>

namespace srs {

class BaseController
{
public:
    BaseController() :
        executingCommand_(Velocity<>()),
        goal_(Pose<>()),
        goalReached_(false),
        Kv_(1.0),
        Kw_(1.0),
        isRobotMoving_(false),
        newCommandAvailable_(false),
        robot_()
    {}

    virtual ~BaseController()
    {}

    Velocity<> getExecutingCommand()
    {
        newCommandAvailable_ = false;
        return executingCommand_;
    }

    Pose<> getGoal() const
    {
        return goal_;
    }

    bool isGoalReached() const
    {
        return goalReached_;
    }

    bool isRobotMoving() const
    {
        return isRobotMoving_;
    }

    bool newCommandAvailable() const
    {
        return newCommandAvailable_;
    }

    virtual void reset()
    {
        goal_ = Pose<>();
        goalReached_ = false;

        isRobotMoving_ = false;

        executingCommand_ = Velocity<>();
        newCommandAvailable_ = false;
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

    void step(Pose<> currentPose, Odometry<> currentOdometry)
    {
        // Perform some basic operations before calling
        // the specific controller step
        isRobotMoving_ = !similarVelocities(currentOdometry.velocity, Velocity<>());

        // Call the controller
        stepController(currentPose, currentOdometry);
    }

protected:
    void executeCommand(Velocity<> command)
    {
        newCommandAvailable_ = true;
        executingCommand_ = command;
    }

    bool similarVelocities(const Velocity<>& lhv, const Velocity<>& rhv)
    {
        // The two velocities to be similar must:
        // - difference in linear velocity must be less than 0.01 m/s
        // - difference in angular velocity must be less than 0.1 deg/s

        // TODO: Move the constants to constexpr
        return abs(lhv.linear - rhv.linear) < 0.01 && abs(lhv.angular - rhv.angular) < 0.002;
    }

    virtual void stepController(Pose<> currentPose, Odometry<> currentOdometry) = 0;

    Velocity<> executingCommand_;

    Pose<> goal_;
    bool goalReached_;

    double Kv_;
    double Kw_;

    bool isRobotMoving_;

    bool newCommandAvailable_;

    RobotProfile robot_;
};

} // namespace srs

#endif // BASECONTROLLER_HPP_
