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
#include <srslib_framework/robotics/controller/StandController.hpp>
#include <srslib_framework/robotics/controller/StopController.hpp>

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
        return activeController_->getExecutingCommand();
    }

    Pose<> getGoal() const
    {
        return activeController_->getGoal();
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
    enum TaskEnum {EMERGENCY_STOP, MANUAL_FOLLOW, NONE, NORMAL_STOP, PATH_FOLLOW, ROTATE, STAND};

    typedef pair<int, Solution<Grid2d>*> WorkType;

    void checkMotionStatus();
    void cleanWorkQueue();

    void executeCommand(bool enforce, CommandEnum command, const Velocity<>* velocity = nullptr);

    bool isManualControllerActive()
    {
        return activeController_ == manualController_;
    }

    bool isPathControllerActive()
    {
        return activeController_ == pathController_;
    }

    bool isRotationControllerActive()
    {
        return activeController_ == rotationController_;
    }

    bool isStandControllerActive()
    {
        return activeController_ == standController_;
    }

    bool isStopControllerActive()
    {
        return activeController_ == stopController_;
    }

    bool isWorkPending()
    {
        return !work_.empty();
    }

    void pumpWorkFromQueue();

    void pushWork(TaskEnum task, Solution<Grid2d>* solution = nullptr)
    {
        work_.push(WorkType(task, solution));
    }

    void selectController(TaskEnum task);

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
    TaskEnum currentTask_;
    Velocity<> currentCommand_;

    double dT_;

    bool emergencyDeclared_;

    ManualController* manualController_;

    ros::NodeHandle rosNodeHandle_;

    CMUPathController* pathController_;
    ros::Publisher pubCmdVel_;

    RobotProfile robot_;
    RotationController* rotationController_;

    StandController* standController_;
    StopController* stopController_;

    queue<WorkType> work_;
};

} // namespace srs

#endif  // MOTIONCONTROLLER_HPP_
