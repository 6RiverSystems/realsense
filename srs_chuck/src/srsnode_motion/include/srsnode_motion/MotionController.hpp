/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MOTIONCONTROLLER_HPP_
#define MOTIONCONTROLLER_HPP_

#include <queue>
using namespace std;

#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/robotics/Odometry.hpp>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/planning/pathplanning/Solution.hpp>

#include <srslib_framework/robotics/controller/CMUPathFollower.hpp>

#include <srslib_framework/robotics/robot/RobotProfile.hpp>

namespace srs {

class MotionController
{
public:
    MotionController(double dT) :
        dT_(dT),
        pathFollower_(nullptr)
    {
        pathFollower_ = new CMUPathFollower();

        reset();
    }

    ~MotionController()
    {}

    void emergencyStop()
    {
        nextTask(TaskEnum::EMERGENCY_STOP);
    }

    void execute(Solution<Grid2d> solution);

    Velocity<> getExecutingCommand()
    {
        if (activeController_)
        {
            return activeController_->getExecutingCommand();
        }

        return Velocity<>();
    }

    Pose<> getGoal()
    {
        if (activeController_)
        {
            return activeController_->getGoal();
        }

        return Pose<>();
    }

    bool isMoving()
    {
        return currentState_ != StateEnum::STANDING;
    }

    bool newCommandAvailable()
    {
        if (activeController_)
        {
            return activeController_->newCommandAvailable();
        }

        return false;
    }

    void normalStop()
    {
        nextTask(TaskEnum::NORMAL_STOP);
    }

    void reset();
    void run(Pose<> currentPose, Odometry<> currentOdometry);

    void setRobot(RobotProfile robot);

private:
    enum TaskEnum {NIL, EMERGENCY_STOP, FOLLOW, NORMAL_STOP, ROTATE, STAND};
    enum StateEnum {NONE, STANDING, FOLLOWING, ROTATING, STOPPING};

    void checkForWork();

    void nextSolution(Solution<Grid2d> nextSolution)
    {
        nextSolution_ = nextSolution;
    }

    void nextState(StateEnum nextState)
    {
        nextState_ = nextState;
    }

    void nextTask(TaskEnum nextTask)
    {
        nextTask_ = nextTask;
    }

    void stateFollowing();
    void stateStanding();
    void stateStopping();

    void taskEmergencyStop();
    void taskFollow();
    void taskNormalStop();
    void taskRotate();
    void taskStand();

    BaseController* activeController_;

    Odometry<> currentOdometry_;
    Pose<> currentPose_;
    Solution<Grid2d> currentSolution_;
    StateEnum currentState_;
    TaskEnum currentTask_;

    double dT_;

    bool emergencyDeclared_;

    CMUPathFollower* pathFollower_;

    Solution<Grid2d> nextSolution_;
    StateEnum nextState_;
    TaskEnum nextTask_;

    RobotProfile robot_;

    queue<pair<int, Solution<Grid2d>>> solutions_;
};

} // namespace srs

#endif  // MOTIONCONTROLLER_HPP_
