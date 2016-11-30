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

#include <srsnode_motion/MotionConfig.h>
using namespace srsnode_motion;

#include <srslib_framework/planning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/trajectory/Trajectory.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/robotics/Odometry.hpp>
#include <srslib_framework/robotics/controller/CMUPathController.hpp>
#include <srslib_framework/robotics/controller/DummyController.hpp>
#include <srslib_framework/robotics/controller/EmergencyController.hpp>
#include <srslib_framework/robotics/controller/ManualController.hpp>
#include <srslib_framework/robotics/controller/RotationController.hpp>
#include <srslib_framework/robotics/controller/StandController.hpp>
#include <srslib_framework/robotics/controller/StopController.hpp>
#include <srslib_framework/robotics/robot_profile/RobotProfile.hpp>

namespace srs {

class MotionController
{
public:
    typedef Solution<Grid2dSolutionItem> SolutionType;

    MotionController(double dT);

    ~MotionController()
    {}

    void emergencyStop();

    void execute(SolutionType solution);

    Velocity<> getExecutingCommand() const
    {
        return activeController_->getExecutingCommand();
    }

    Pose<> getFinalGoal() const
    {
        return currentFinalGoal_;
    }

    void getLanding(vector<Pose<>>& landingArea) const
    {
        activeController_->getLanding(landingArea);
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

    bool isMoving()
    {
        return isMovingControllerActive();
    }

    void normalStop();

    void reset();
    void run(Pose<> currentPose, Odometry<> currentOdometry, Velocity<> currentCommand);

    void setRobotProfile(RobotProfile robotProfile);
    void switchToManual();
    void switchToAutonomous();

private:
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

    typedef pair<int, SolutionType*> WorkType;

    void aggregateInitialRotations(Pose<> pose, SolutionType*& solution, SolutionType*& rotation);
    void aggregateFinalRotations(SolutionType*& solution, SolutionType*& rotation);

    void checkMotionStatus();
    void cleanWorkQueue();

    void executeCommand(bool enforce, CommandEnum command, const Velocity<>* velocity = nullptr);
    void executeFinalRotation();

    bool isDummyControllerActive()
    {
        return activeController_ == dummyController_;
    }

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

    bool isScheduledNext(TaskEnum task)
    {
        WorkType nextTask = work_.front();
        return nextTask.first == task;
    }

    bool isScheduled(TaskEnum task)
    {
        for (auto workItem : work_)
        {
            if (workItem.first == task)
            {
                return true;
            }
        }

        return false;
    }

    string printWorkToString();
    void popWorkItem(TaskEnum& task, SolutionType& solution);
    void prependWorkItem(TaskEnum task, SolutionType* solution = nullptr);
    void publishLookAheadDistance();
    void pumpWorkFromQueue();
    void pushWorkItem(TaskEnum task, SolutionType* solution = nullptr);
    Pose<> pushWorkSolution(Pose<> initialPose, SolutionType& solution);

    void selectController(TaskEnum task);

    void setHasArrived(bool newValue)
    {
        if (newValue != hasArrived_)
        {
            hasArrived_ = newValue;
            hasArrivedChanged_ = true;
        }
    }

    void taskEmergencyStop();
    void taskManualFollow();
    void taskNone();
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
    SolutionType currentSolution_;
    TaskEnum currentTask_;

    double dT_;
    DummyController* dummyController_;

    EmergencyController* emergencyController_;

    bool hasArrived_;
    bool hasArrivedChanged_;

    ManualController* manualController_;

    CMUPathController* pathController_;
    ros::Publisher pubCmdVel_;
    ros::Publisher pubLookAheadDistance_;

    RobotProfile robot_;
    ros::NodeHandle rosNodeHandle_;
    RotationController* rotationController_;

    StandController* standController_;
    StopController* stopController_;

    deque<WorkType> work_;
};

} // namespace srs

#endif  // MOTIONCONTROLLER_HPP_
