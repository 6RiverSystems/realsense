#include <srsnode_motion/MotionController.hpp>

#include <ros/ros.h>

#include <srslib_framework/math/PoseMath.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MotionController::MotionController(double lookAheadDistance, double distanceToGoal) :
    distanceToGoal_(distanceToGoal),
    lookAheadDistance_(lookAheadDistance),
    lowLevelController_(1, 1)
{
    reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::reset()
{
    moving_ = false;
    newCommandAvailable_ = false;
    executingCommand_ = Velocity<>();

    currentRobotPose_ = Pose<>();
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

    updateProjectionIndex();

    Velocity<> nextVelocity;

    double distance = PoseMath::euclidean(currentRobotPose_, goal_);
    if (distance < distanceToGoal_)
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
        trajectory_.getVelocity(projectionIndex_, nextVelocity);

        // Ask the low-level controller to modulate the desired velocity
        nextVelocity = lowLevelController_.step(currentRobotPose_, nextVelocity);

        setExecutingCommand(nextVelocity);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::setTrajectory(Trajectory<> trajectory)
{
    trajectory_ = trajectory;

    if (!trajectory_.empty())
    {
        cout << trajectory_ << endl;

        if (isMoving())
        {
            stop(0.0);
        }
        reset();

        // Find the goal of the trajectory
        trajectory_.getGoal(goal_);

        // Find the first reference point on the given trajectory
        referenceIndex_ = trajectory_.findClosestPose(currentRobotPose_);
        trajectory_.getPose(referenceIndex_, referencePose_);

        // Set the reference in the low-level controller
        lowLevelController_.setReference(referencePose_);

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
    int deltaIndex = round(lookAheadDistance_ / trajectory_.getDeltaDistance());
    int trajectorySize = trajectory_.size();

    int searchIndex = projectionIndex_ + 2 * deltaIndex;
    if (searchIndex > trajectorySize)
    {
        searchIndex = trajectorySize - 1;
    }

    projectionIndex_ = trajectory_.findClosestPose(currentRobotPose_,
        projectionIndex_, searchIndex);

    referenceIndex_ = projectionIndex_ + deltaIndex;
    if (referenceIndex_ > trajectorySize)
    {
        referenceIndex_ = trajectorySize - 1;
    }

    trajectory_.getPose(referenceIndex_, referencePose_);
    lowLevelController_.setReference(referencePose_);

    ROS_INFO_STREAM("Switching to reference pose [" << referenceIndex_ << "] " << referencePose_);
}

} // namespace srs
