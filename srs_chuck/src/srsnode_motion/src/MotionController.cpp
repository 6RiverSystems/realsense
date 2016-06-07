#include <srsnode_motion/MotionController.hpp>

#include <ros/ros.h>

#include <srslib_framework/math/PoseMath.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MotionController::MotionController() :
    robot_(),
    lowLevelController_(1, 1)
{
    reset(Pose<>());
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::reset(Pose<> robotPose)
{
    moving_ = false;
    newCommandAvailable_ = false;
    executingCommand_ = Velocity<>();

    currentRobotPose_ = robotPose;
    projectionIndex_ = -1;

    referencePose_ = Pose<>();
    referenceIndex_ = -1;

    goal_ = Pose<>();
    goalReached_ = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::run(double dT, Pose<> robotPose)
{
    currentRobotPose_ = robotPose;
    ROS_INFO_STREAM("Reported pose: " << currentRobotPose_);

    if (moving_)
    {
        updateProjectionIndex();

        Velocity<> nextCommand;

        double distance = PoseMath::euclidean(currentRobotPose_, goal_);
        ROS_INFO_STREAM("Distance to goal: " << distance);

        if (distance < robot_.goalReachedDistance)
        {
            goalReached_ = true;
            ROS_INFO_STREAM("Goal reached: " << goal_);

            stop(0.0);
        }
        else
        {
            distance = PoseMath::euclidean(currentRobotPose_, referencePose_);
            ROS_INFO_STREAM("Distance to reference: " << distance);

            // Find the desired velocity command
            nextCommand = trajectory_.getVelocity(projectionIndex_);

            // Ask the low-level controller to modulate the desired velocity
            nextCommand = lowLevelController_.step(currentRobotPose_, nextCommand);

            // If the two velocities are similar, there is no need to send a
            // new command (if requested).
            if (!similarVelocities(executingCommand_, nextCommand))
            {
                executingCommand_ = nextCommand;
                newCommandAvailable_ = true;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::setRobot(RobotProfile robot)
{
    robot_ = robot;

    lowLevelController_.setLookAheadDistance(robot_.lookAheadDistance);
    lowLevelController_.setMaxAngularVelocity(robot_.maxAngularVelocity);
    lowLevelController_.setMaxLinearVelocity(robot_.maxLinearVelocity);
    lowLevelController_.setTravelAngularVelocity(robot_.travelAngularVelocity);
    lowLevelController_.setTravelLinearVelocity(robot_.travelLinearVelocity);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::setTrajectory(Trajectory<> trajectory)
{
    trajectory_ = trajectory;

    if (!trajectory_.empty())
    {
        if (isMoving())
        {
            stop(0.0);
        }
        reset(currentRobotPose_);

        cout << currentRobotPose_ << endl;

        // Find the goal of the trajectory
        goal_ = trajectory_.getGoal();

        // Find the first reference point on the given trajectory
        referenceIndex_ = trajectory_.findClosestPose(currentRobotPose_);
        referencePose_ = trajectory_.getPose(referenceIndex_);

        // Set the reference in the low-level controller
        lowLevelController_.setReference(referencePose_);

        // Find the projection of the current robot pose onto the trajectory
        projectionIndex_ = trajectory_.findClosestPose(currentRobotPose_,
            0, robot_.lookAheadDistance);

        moving_ = true;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::stop(double stopDistance)
{
    // TODO: Make the stop command function with the stop distance
    newCommandAvailable_ = true;
    executingCommand_ = Velocity<>();

    moving_ = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::updateProjectionIndex()
{
    projectionIndex_ = trajectory_.findClosestPose(currentRobotPose_,
        projectionIndex_, robot_.lookAheadDistance);
    referenceIndex_ = trajectory_.findWaypointAtDistance(projectionIndex_,
        robot_.lookAheadDistance);

    referencePose_ = trajectory_.getPose(referenceIndex_);
    lowLevelController_.setReference(referencePose_);
}

} // namespace srs
