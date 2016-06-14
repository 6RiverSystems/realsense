/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MOTIONCONTROLLER_HPP_
#define MOTIONCONTROLLER_HPP_

#include <queue>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/robotics/Odometry.hpp>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/planning/pathplanning/Solution.hpp>

#include <srslib_framework/robotics/controller/CMUPathController.hpp>
#include <srslib_framework/robotics/controller/ManualController.hpp>
#include <srslib_framework/robotics/controller/RotationController.hpp>

#include <srslib_framework/robotics/RobotProfile.hpp>

namespace srs {

class MotionController
{
public:
    MotionController(double dT);

    ~MotionController()
    {}

    void emergencyStop();

    void execute(Solution<Grid2d> solution);

    Velocity<> getExecutingCommand() const
    {
        if (activeController_)
        {
            return activeController_->getExecutingCommand();
        }

        return Velocity<>();
    }

    Pose<> getGoal() const
    {
        if (activeController_)
        {
            return activeController_->getGoal();
        }

        return Pose<>();
    }

    bool isMoving() const
    {
        return currentState_ != StateEnum::STANDING || currentTask_ != TaskEnum::STAND;
    }

    void normalStop();

    void reset();
    void run(Pose<> currentPose, Odometry<> currentOdometry, Velocity<> currentCommand);

    void setRobot(RobotProfile robot);
    void switchToManual();
    void switchToAutonomous();

private:
    const static Velocity<> ZERO_VELOCITY;

    enum CommandEnum {VELOCITY, E_STOP};
    enum StateEnum {NIL, STANDING, RUNNING, ROTATING, STOPPING};
    enum TaskEnum {NONE, EMERGENCY_STOP, PATH_FOLLOW, MANUAL_FOLLOW, NORMAL_STOP, ROTATE, STAND};

    typedef pair<int, Solution<Grid2d>*> WorkType;

    void checkForWork();

    void executeCommand(CommandEnum command, const Velocity<>* velocity = nullptr);

    void nextState(StateEnum nextState)
    {
        nextState_ = nextState;
    }

    void nextTask(TaskEnum nextTask)
    {
        nextTask_ = nextTask;
    }

    void pushWork(TaskEnum task, Solution<Grid2d>* solution = nullptr)
    {
        work_.push(WorkType(task, solution));
    }

    void selectController(TaskEnum task);
    void stateRunning();
    void stateStanding();
    void stateStopping();

    void taskEmergencyStop();
    void taskManualFollow();
    void taskNormalStop();
    void taskPathFollow();
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

    ManualController* manualController_;

    ros::NodeHandle rosNodeHandle_;

    Solution<Grid2d> nextSolution_;
    StateEnum nextState_;
    TaskEnum nextTask_;

    CMUPathController* pathController_;
    ros::Publisher pubCmdVel_;

    RobotProfile robot_;
    RotationController* rotationController_;

    queue<WorkType> work_;
};

} // namespace srs

#endif  // MOTIONCONTROLLER_HPP_
