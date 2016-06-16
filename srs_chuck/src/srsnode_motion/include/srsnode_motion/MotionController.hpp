/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MOTIONCONTROLLER_HPP_
#define MOTIONCONTROLLER_HPP_

#include <deque>
#include <string>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/robotics/Odometry.hpp>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/planning/pathplanning/Solution.hpp>

#include <srslib_framework/robotics/controller/CMUPathController.hpp>
#include <srslib_framework/robotics/controller/EmergencyController.hpp>
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

    Pose<> getFinalGoal() const
    {
        return currentFinalGoal_;
    }

    Pose<> getShortTermGoal() const
    {
        return currentShortTermGoal_;
    }

    bool hasArrived() const
    {
        return hasArrived_;
    }

    bool hasArrivedChanged() const
    {
        return hasArrivedChanged_;
    }

    bool isEmergencyDeclared()
    {
        return isEmergencyControllerActive();
    }

    void normalStop();

    void reset();
    void run(Pose<> currentPose, Odometry<> currentOdometry, Velocity<> currentCommand);

    void setRobot(RobotProfile robot);
    void switchToManual();
    void switchToAutonomous();

private:
    const static Velocity<> ZERO_VELOCITY;

    enum CommandEnum {
        BRAKE,
        VELOCITY
    };

    enum TaskEnum {
        EMERGENCY_STOP,
        MANUAL_FOLLOW,
        NONE, NORMAL_STOP,
        PATH_FOLLOW,
        ROTATE,
        STAND
    };

    static unordered_map<int, string> TASK_NAMES;

    typedef pair<int, Solution<Grid2d>*> WorkType;

    void checkMotionStatus();
    void cleanWorkQueue();

    void executeCommand(bool enforce, CommandEnum command, const Velocity<>* velocity = nullptr);

    bool isEmergencyControllerActive()
    {
        return activeController_ == emergencyController_;
    }

    bool isManualControllerActive()
    {
        return activeController_ == manualController_;
    }

    bool isMovingControllerActive()
    {
        return isPathControllerActive() || isRotationControllerActive();
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

    string printWorkToString();

    void pumpWorkFromQueue();

    void pushWork(TaskEnum task, Solution<Grid2d>* solution = nullptr)
    {
        ROS_INFO_STREAM_NAMED("MotionController", printWorkToString());

        ROS_INFO_STREAM_NAMED("MotionController", "Pushing task: " << TASK_NAMES[task]);
        if (solution)
        {
            ROS_INFO_STREAM_NAMED("MotionController", "Pushing solution: " << *solution);
        }
        else
        {
            ROS_INFO_STREAM_NAMED("MotionController", "Pushing solution: (null)");
        }

        work_.push_back(WorkType(task, solution));
    }

    void selectController(TaskEnum task);

    void setHasArrived(bool newValue)
    {
        hasArrived_ = newValue;
        hasArrivedChanged_ = true;
    }

    void taskEmergencyStop();
    void taskManualFollow();
    void taskNormalStop();
    void taskPathFollow();
    void taskRotate();
    void taskStand();

    BaseController* activeController_;

    Velocity<> currentCommand_;
    Pose<> currentFinalGoal_;
    Odometry<> currentOdometry_;
    Pose<> currentPose_;
    Pose<> currentShortTermGoal_;
    Solution<Grid2d> currentSolution_;
    TaskEnum currentTask_;

    double dT_;

    EmergencyController* emergencyController_;

    bool hasArrived_;
    bool hasArrivedChanged_;

    ManualController* manualController_;

    CMUPathController* pathController_;
    ros::Publisher pubCmdVel_;

    RobotProfile robot_;
    ros::NodeHandle rosNodeHandle_;
    RotationController* rotationController_;

    StandController* standController_;
    StopController* stopController_;

    deque<WorkType> work_;
};

} // namespace srs

#endif  // MOTIONCONTROLLER_HPP_
