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
    MotionController() :
        pathFollower_(1, 1)
    {
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
        return currentState_ != StateEnum::STANDING;
    }

    bool newCommandAvailable()
    {
        return newCommandAvailable_;
    }

    void normalStop()
    {
        nextTask(TaskEnum::NORMAL_STOP);
    }

    void reset();
    void run(Pose<> robotPose, Odometry<> odometry);

    void setRobot(RobotProfile robot);

private:
    enum TaskEnum {NIL, EMERGENCY_STOP, FOLLOW, NORMAL_STOP, ROTATE, STAND};
    enum StateEnum {NONE, STANDING, FOLLOWING, ROTATING, STOPPING};

    void checkForWork();

    void executeCommand(Velocity<> command)
    {
        newCommandAvailable_ = true;
        executingCommand_ = command;
    }

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

    // TODO: find a better place for this
    bool similarVelocities(const Velocity<>& lhv, const Velocity<>& rhv)
    {
        // The two velocities to be similar must:
        // - difference in linear velocity must be less than 0.01 m/s
        // - difference in angular velocity must be less than 0.1 deg/s
        return abs(lhv.linear - rhv.linear) < 0.01 && abs(lhv.angular - rhv.angular) < 0.002;
    }

    void stateFollowing();
    void stateStanding();
    void stateStopping();

    void taskEmergencyStop();
    void taskFollow();
    void taskNormalStop();
    void taskRotate();
    void taskStand();

    void updateProjectionIndex();

    Odometry<> currentOdometry_;
    Pose<> currentRobotPose_;
    Solution<Grid2d> currentSolution_;
    StateEnum currentState_;
    TaskEnum currentTask_;
    Trajectory<> currentTrajectory_;

    bool emergencyDeclared_;
    Velocity<> executingCommand_;

    Pose<> goal_;
    bool goalReached_;

    CMUPathFollower pathFollower_;

    bool newCommandAvailable_;
    Solution<Grid2d> nextSolution_;
    StateEnum nextState_;
    TaskEnum nextTask_;

    int projectionIndex_;

    Pose<> referencePose_;
    int referenceIndex_;
    RobotProfile robot_;

    queue<pair<int, Solution<Grid2d>>> solutions_;
};

} // namespace srs

#endif  // MOTIONCONTROLLER_HPP_
