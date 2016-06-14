#include <srslib_framework/robotics/controller/CMUPathController.hpp>

#include <ros/ros.h>

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/BasicMath.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void CMUPathController::reset()
{
    BaseController::reset();

    dynamicLookAheadDistance_ = robot_.maxLookAheadDistance;

    currentTrajectory_.clear();

    projectionIndex_ = -1;
    referencePose_ = Pose<>();
    referenceIndex_ = -1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void CMUPathController::setTrajectory(Trajectory<> trajectory, Pose<> robotPose)
{
    currentTrajectory_ = trajectory;
    if (!currentTrajectory_.empty())
    {
        // Find the goal of the trajectory
        goal_ = currentTrajectory_.getGoal();
        goalReached_ = false;

        // Find the first reference point on the given trajectory
        referenceIndex_ = currentTrajectory_.findClosestPose(robotPose);
        referencePose_ = currentTrajectory_.getPose(referenceIndex_);

        // Find the projection of the current robot pose onto the trajectory
        projectionIndex_ = currentTrajectory_.findClosestPose(robotPose, 0,
            robot_.maxLookAheadDistance);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Protected methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void CMUPathController::stepController(double dT, Pose<> currentPose, Odometry<> currentOdometry)
{
    // Update the projection of the current robot pose onto the path
    updateProjectionIndex(currentPose);

    double distanceToGoal = PoseMath::euclidean(currentPose, goal_);
    ROS_DEBUG_STREAM_NAMED("CMUPathController", "Distance to goal: " << distanceToGoal);

    double linear = 0.0;
    double angular = 0.0;

    if (canceled_ || distanceToGoal < robot_.goalReachedDistance)
    {
        goalReached_ = true;
        return;
    }

    // Find the desired velocity command
    Velocity<> command = currentTrajectory_.getVelocity(projectionIndex_);

    // Calculate the linear portion of the command
    linear = Kv_ * command.linear;
    linear = BasicMath::saturate<double>(linear,
        robot_.maxLinearVelocity, -robot_.maxLinearVelocity);

    // Calculate the angular portion of the command
    double slope = atan2(referencePose_.y - currentPose.y, referencePose_.x - currentPose.x);
    double alpha = AngleMath::normalizeAngleRad<double>(slope - currentPose.theta);

    angular = Kw_ * 2 * sin(alpha) / dynamicLookAheadDistance_;

    // If the robot angle is greater than 45deg, use a different rotation velocity
    if (abs(alpha) > M_PI / 4)
    {
        angular = BasicMath::sgn(angular) * robot_.travelRotationVelocity;
    }

    angular = BasicMath::saturate<double>(angular,
        robot_.maxAngularVelocity, -robot_.maxAngularVelocity);

    // Send the command for execution
    executeCommand(Velocity<>(linear, angular));

    updateLookAheadDistance();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void CMUPathController::updateLookAheadDistance()
{
    // Recalculate the look-ahead distance based roughly on the velocity
    dynamicLookAheadDistance_ = BasicMath::saturate(
        executingCommand_.linear * robot_.ratioLookAheadDistance,
        robot_.maxLookAheadDistance, robot_.minLookAheadDistance);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void CMUPathController::updateProjectionIndex(Pose<> robotPose)
{
    projectionIndex_ = currentTrajectory_.findClosestPose(robotPose,
        projectionIndex_, dynamicLookAheadDistance_);

    referenceIndex_ = currentTrajectory_.findWaypointAtDistance(projectionIndex_,
        dynamicLookAheadDistance_);

    referencePose_ = currentTrajectory_.getPose(referenceIndex_);
}

} // namespace srs
