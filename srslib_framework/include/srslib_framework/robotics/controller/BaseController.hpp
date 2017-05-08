/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef BASECONTROLLER_HPP_
#define BASECONTROLLER_HPP_

#include <vector>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/VelocityMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Odometry.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/robotics/robot_profile/RobotProfile.hpp>

namespace srs {

class BaseController
{
public:
    BaseController(string name) :
        name_(name)
    {}

    virtual ~BaseController()
    {}

    Velocity<> getExecutingCommand() const
    {
        return executingCommand_;
    }

    Pose<> getGoal() const
    {
        return goal_;
    }

    void getLanding(vector<Pose<>>& landingArea) const
    {
        landingArea = goalLanding_;
    }

    string getName() const
    {
        return name_;
    }

    bool isTerminated() const
    {
        return terminated_;
    }

    bool isFirstRun() const
    {
        return firstRun_;
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
        calculateLanding(goal);
    }

    void setGoalReached(const bool newValue)
    {
        goalReached_ = newValue;
    }

    void step(double dT, Pose<> currentPose, Odometry<> currentOdometry);
    virtual void setRobotProfile(RobotProfile robotProfile) = 0;

    void terminate()
    {
        terminated_ = true;

        ROS_DEBUG_STREAM_NAMED("base_controller", name_ << " terminated");
    }

protected:
    virtual void calculateLanding(Pose<> goal)
    {}

    bool checkGoalReached(Pose<> currentPose);

    void executeCommand(Velocity<> command);
    void executeCommand(double linear, double angular);

    virtual void stepController(double dT, Pose<> currentPose, Odometry<> currentOdometry) = 0;

    void setRobotProfile(RobotProfile robotProfile, double Kv, double Kw)
    {
        robot_ = robotProfile;

        Kv_ = Kv;
        Kw_ = Kw;
    }

    RobotProfile    robot_{ };
    vector<Pose<>>  goalLanding_{ };

private:

    bool            terminated_{ false };

    Velocity<>      executingCommand_{ Velocity<>::ZERO };

    bool            firstRun_{ true };

    Pose<>          goal_{ Pose<>::ZERO };
    bool            goalReached_{ false };

    bool            isRobotMoving_{ false };

    double          Kv_{ 1.0 };
    double          Kw_{ 1.0 };

    string          name_{ "" };
};

} // namespace srs

#endif // BASECONTROLLER_HPP_
