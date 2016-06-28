#include <srslib_framework/robotics/controller/CMUPathController.hpp>

#include <ros/ros.h>

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void CMUPathController::reset()
{
    BaseController::reset();

    lookAheadDistance_ = robot_.pathFollowZeroLookAheadDistance;

    currentVelocity_ = 0.0;
    velocityChange_ = 0.0;
    velocityChangePose_ = ZERO_POSE;
    velocityCurrentMax_ = 0.0;
    currentTrajectory_.clear();

    projectionIndex_ = -1;
    referencePose_ = ZERO_POSE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void CMUPathController::setTrajectory(Trajectory<> trajectory, Pose<> robotPose)
{
    reset();

    currentTrajectory_ = trajectory;
    if (!currentTrajectory_.empty())
    {
        // Find the goal of the trajectory
        setGoal(currentTrajectory_.getGoal());

        // Find the first reference point on the given trajectory
        int referenceIndex = currentTrajectory_.findClosestPose(robotPose);
        referencePose_ = currentTrajectory_.getPose(referenceIndex);

        // Find the projection of the current robot pose onto the trajectory
        projectionIndex_ = currentTrajectory_.findClosestPose(robotPose, 0,
            robot_.pathFollowMaxLookAheadDistance);

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
void CMUPathController::calculateLanding(Pose<> goal)
{
    Pose<> reflection = PoseMath::rotate(goal, M_PI);
    Pose<> p = PoseMath::transform(reflection, robot_.pathFollowGoalReachedDistance);

    reflection = PoseMath::rotate(p, M_PI_2);
    Pose<> p0 = PoseMath::transform(reflection, robot_.pathFollowLandingWidth / 2);

    reflection = PoseMath::rotate(p, -M_PI_2);
    Pose<> p1 = PoseMath::transform(reflection, robot_.pathFollowLandingWidth / 2);

    reflection = p1;
    reflection.theta = goal.theta;
    Pose<> p2 = PoseMath::transform(reflection, robot_.pathFollowLandingDepth);

    reflection = p0;
    reflection.theta = goal.theta;
    Pose<> p3 = PoseMath::transform(reflection, robot_.pathFollowLandingDepth);

    goalLanding_.clear();
    goalLanding_.push_back(p0);
    goalLanding_.push_back(p1);
    goalLanding_.push_back(p2);
    goalLanding_.push_back(p3);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void CMUPathController::stepController(double dT, Pose<> currentPose, Odometry<> currentOdometry)
{
    // If the controller has no work to do, or it has received a cancel signal or
    // the distance to the goal is smaller than the threshold
    // declare that the goal was reached
    if (currentTrajectory_.empty() || isCanceled() || checkGoalReached(currentPose))
    {
        setGoalReached(true);
        executeCommand(ZERO_VELOCITY);

        return;
    }

    // Update the projection of the current robot pose onto the path and the
    // look-ahead distance
    updateParameters(currentPose);

    double distanceToReference = PoseMath::euclidean(currentPose, referencePose_);
    ROS_DEBUG_STREAM_NAMED("CMUPathController", "Distance to reference: " << distanceToReference);

    double distanceToChange = PoseMath::euclidean(currentPose, velocityChangePose_);
    ROS_DEBUG_STREAM_NAMED("CMUPathController", "Distance to change: " << distanceToChange);

    // Calculate what would be the velocity change based on the set acceleration
    double deltaVelocity = robot_.pathFollowLinearAcceleration * dT;

    // Calculate the velocity after the acceleration has taken place
    double linear = currentVelocity_ + deltaVelocity;

    // Calculate the stopping distance based on the projected velocity
    // and the velocity requested at the velocity change point
    double changingTime = (velocityChange_ - linear) / robot_.pathFollowLinearAcceleration;
    double changingDistance = 0.5 * (velocityChange_ + linear) * abs(changingTime);

    ROS_DEBUG_STREAM_NAMED("CMUPathController", "Current max velocity: " << velocityCurrentMax_);
    ROS_DEBUG_STREAM_NAMED("CMUPathController", "Next velocity change: " << velocityChange_);
    ROS_DEBUG_STREAM_NAMED("CMUPathController", "Changing time: " << changingTime);
    ROS_DEBUG_STREAM_NAMED("CMUPathController", "Changing distance: " << changingDistance);

    // If the robot is sufficiently far away from the changing point
    // the velocity can be increased. Otherwise, the deceleration must commence
    if (distanceToChange > changingDistance)
    {
        // Enforce the current maximum velocity
        linear = BasicMath::saturate<double>(linear, velocityCurrentMax_, 0.0);
        currentVelocity_ = linear;

        ROS_DEBUG_STREAM_NAMED("CMUPathController",
            "Free increase velocity to: " << linear);
    }
    else
    {
        if (currentVelocity_ > velocityChange_)
        {
            // Reduce the moving velocity, if the new requested
            // velocity is less than the current one
            currentVelocity_ = BasicMath::threshold<double>(currentVelocity_ - deltaVelocity,
                robot_.pathFollowMinLinearVelocity, 0.0);

            ROS_DEBUG_STREAM_NAMED("CMUPathController",
                "Decreasing velocity: " << currentVelocity_);
        }
        else
        {
            // Increase the velocity to match if the new requested
            // velocity is greater than the current one
            linear = BasicMath::saturate<double>(linear, velocityCurrentMax_, 0.0);
            currentVelocity_ = linear;

            ROS_DEBUG_STREAM_NAMED("CMUPathController",
                "Increasing velocity: " << linear);
        }
    }

    // Calculate the angular portion of the command
    double slope = atan2(referencePose_.y - currentPose.y, referencePose_.x - currentPose.x);
    double alpha = AngleMath::normalizeAngleRad<double>(slope - currentPose.theta);

    double angular = 2 * sin(alpha) / lookAheadDistance_;

    // Make sure that the calculated angular velocity is not bigger
    // than the specified travel angular velocity
    angular = BasicMath::saturate<double>(angular,
        robot_.pathFollowMaxAngularVelocity, -robot_.pathFollowMaxAngularVelocity);

    // Send the command for execution
    executeCommand(currentVelocity_, angular);
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
        if (!BasicMath::equal(fromVelocity, velocity, robot_.physicalMinLinearVelocity))
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
    if (robot_.pathFollowAdaptiveLookAhead)
    {
        Pose<> projectionPose = currentTrajectory_.getPose(projectionIndex_);

        lookAheadDistance_ = robot_.pathFollowZeroLookAheadDistance +
            PoseMath::euclidean(currentPose, projectionPose);

        lookAheadDistance_ = BasicMath::saturate(lookAheadDistance_,
            robot_.pathFollowMaxLookAheadDistance, robot_.pathFollowMinLookAheadDistance);

        ROS_DEBUG_STREAM_NAMED("CMUPathController", "Look-ahead distance: " << lookAheadDistance_);
    }
}

} // namespace srs
