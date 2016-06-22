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

    lookAheadDistance_ = robot_.zeroLookAheadDistance;

    currentVelocity_ = 0.0;
    velocityChange_ = 0.0;
    velocityChangePose_ = Pose<>();
    velocityCurrentMax_ = 0.0;
    currentTrajectory_.clear();

    projectionIndex_ = -1;
    referencePose_ = Pose<>();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void CMUPathController::setTrajectory(Trajectory<> trajectory, Pose<> robotPose)
{
    reset();

    currentTrajectory_ = trajectory;
    if (!currentTrajectory_.empty())
    {
        // Find the goal of the trajectory
        goal_ = currentTrajectory_.getGoal();
        goalReached_ = false;

        // Find the first reference point on the given trajectory
        int referenceIndex = currentTrajectory_.findClosestPose(robotPose);
        referencePose_ = currentTrajectory_.getPose(referenceIndex);

        // Find the projection of the current robot pose onto the trajectory
        projectionIndex_ = currentTrajectory_.findClosestPose(robotPose, 0,
            robot_.maxLookAheadDistance);

        // Find the first maximum velocity change
        velocityChange_ = getMaxVelocity(projectionIndex_);
        velocityCurrentMax_ = velocityChange_;
        int velocityChangeIndex = findVelocityChange(velocityChange_, projectionIndex_);
        velocityChangePose_ = currentTrajectory_.getPose(velocityChangeIndex);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Protected methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void CMUPathController::stepController(double dT, Pose<> currentPose, Odometry<> currentOdometry)
{
    // Update the projection of the current robot pose onto the path and the
    // look-ahead distance
    updateParameters(currentPose);

    double distanceToGoal = PoseMath::euclidean(currentPose, goal_);
    ROS_DEBUG_STREAM_NAMED("CMUPathController", "Distance to goal: " << distanceToGoal);

    double distanceToReference = PoseMath::euclidean(currentPose, referencePose_);
    ROS_DEBUG_STREAM_NAMED("CMUPathController", "Distance to reference: " << distanceToReference);

    double distanceToChange = PoseMath::euclidean(currentPose, velocityChangePose_);
    ROS_DEBUG_STREAM_NAMED("CMUPathController", "Distance to change: " << distanceToChange);

    // If the controller has received a cancel signal or
    // the distance to the goal is smaller than the threshold
    // declare that the goal was reached
    if (canceled_ || distanceToGoal < robot_.goalReachedDistance)
    {
        goalReached_ = true;
        executeCommand(Velocity<>(0.0, 0.0));

        return;
    }

    // Calculate what would be the velocity change based on the set acceleration
    double deltaVelocity = Kv_ * robot_.travelLinearAcceleration * dT;

    // Calculate the velocity after the acceleration has taken place
    double projectedVelocity = currentVelocity_ + deltaVelocity;

    // Calculate the stopping distance based on the projected velocity
    // and the velocity requested at the velocity change point
    double changingTime = (velocityChange_ - projectedVelocity) / robot_.travelLinearAcceleration;
    double changingDistance = 0.5 * (velocityChange_ + projectedVelocity) * abs(changingTime);

    ROS_DEBUG_STREAM_NAMED("CMUPathController", "Current max velocity: " << velocityCurrentMax_);
    ROS_DEBUG_STREAM_NAMED("CMUPathController", "Next velocity change: " << velocityChange_);
    ROS_DEBUG_STREAM_NAMED("CMUPathController", "Changing time: " << changingTime);
    ROS_DEBUG_STREAM_NAMED("CMUPathController", "Changing distance: " << changingDistance);

    // If the robot is sufficiently far away from the changing point
    // the velocity can be increased. Otherwise, the deceleration must commence
    if (distanceToChange > changingDistance)
    {
        projectedVelocity = BasicMath::saturate<double>(projectedVelocity, velocityCurrentMax_, 0.0);
        currentVelocity_ = projectedVelocity;

        ROS_DEBUG_STREAM_NAMED("CMUPathController",
            "Free increase velocity to: " << projectedVelocity);
    }
    else
    {
        if (currentVelocity_ > velocityChange_)
        {
            currentVelocity_ = BasicMath::threshold<double>(currentVelocity_ - deltaVelocity,
                robot_.minLinearVelocity, 0.0);
            ROS_DEBUG_STREAM_NAMED("CMUPathController",
                "Decreasing velocity: " << currentVelocity_);
        }
        else
        {
            projectedVelocity = BasicMath::saturate<double>(projectedVelocity, velocityCurrentMax_, 0.0);
            currentVelocity_ = projectedVelocity;

            ROS_DEBUG_STREAM_NAMED("CMUPathController",
                "Increasing velocity: " << projectedVelocity);
        }
    }

    // Calculate the angular portion of the command
    double slope = atan2(referencePose_.y - currentPose.y, referencePose_.x - currentPose.x);
    double alpha = AngleMath::normalizeAngleRad<double>(slope - currentPose.theta);

    double angular = Kw_ * 2 * sin(alpha) / lookAheadDistance_;

    angular = BasicMath::saturate<double>(angular,
        robot_.maxAngularVelocity, -robot_.maxAngularVelocity);

    angular = BasicMath::threshold<double>(angular,
        robot_.minPhysicalAngularVelocity, 0.0);

    // Send the command for execution
    executeCommand(Velocity<>(currentVelocity_, angular));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
int CMUPathController::findVelocityChange(double fromVelocity, int fromIndex)
{
    if (fromIndex >= currentTrajectory_.size())
    {
        return currentTrajectory_.size() - 1;
    }

    for (int i = fromIndex; i < currentTrajectory_.size(); i++)
    {
        double velocity = getMaxVelocity(i);
        if (!BasicMath::equal(fromVelocity, velocity, robot_.minPhysicalLinearVelocity))
        {
            return i;
        }
    }

    return currentTrajectory_.size() - 1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
double CMUPathController::getMaxVelocity(int position)
{
    TrajectoryAnnotation annotation = currentTrajectory_.getAnnotation(position);
    return annotation.maxVelocity;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void CMUPathController::updateParameters(Pose<> currentPose)
{
    // Update the projection on the trajectory
    // based on the current pose
    projectionIndex_ = currentTrajectory_.findClosestPose(currentPose,
        projectionIndex_, lookAheadDistance_);

    // Find a new reference point at the
    // look-ahead distance from the current projection
    int referenceIndex = currentTrajectory_.findWaypointAtDistance(projectionIndex_,
        lookAheadDistance_);

    // Look up the pose at the reference point
    referencePose_ = currentTrajectory_.getPose(referenceIndex);

    // Update the next maximum velocity change
    velocityCurrentMax_ = getMaxVelocity(projectionIndex_);
    int velocityChangeIndex = findVelocityChange(velocityCurrentMax_, projectionIndex_);
    velocityChange_ = getMaxVelocity(velocityChangeIndex);
    velocityChangePose_ = currentTrajectory_.getPose(velocityChangeIndex);

    // Recalculate the look-ahead distance
    if (robot_.adaptiveLookAhead)
    {
        Pose<> projectionPose = currentTrajectory_.getPose(projectionIndex_);

        lookAheadDistance_ = robot_.zeroLookAheadDistance +
            PoseMath::euclidean(currentPose, projectionPose);

        lookAheadDistance_ = BasicMath::saturate(lookAheadDistance_,
            robot_.maxLookAheadDistance, robot_.minLookAheadDistance);

        ROS_DEBUG_STREAM_NAMED("CMUPathController", "Look-ahead distance: " << lookAheadDistance_);
    }
}

} // namespace srs
