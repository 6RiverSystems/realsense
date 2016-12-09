#include <srslib_framework/robotics/controller/BaseController.hpp>

#include <srslib_framework/math/PoseMath.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void BaseController::reset()
{
    terminated_ = false;

    firstRun_ = true;

    goal_ = Pose<>::ZERO;
    goalReached_ = false;

    isRobotMoving_ = false;

    executingCommand_ = Velocity<>::ZERO;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void BaseController::step(double dT, Pose<> currentPose, Odometry<> currentOdometry)
{
    // Perform some basic operations before calling
    // the specific controller step
    isRobotMoving_ = !VelocityMath::equal<double>(currentOdometry.velocity, Velocity<>::ZERO);

    // Call the controller
    stepController(dT, currentPose, currentOdometry);

    firstRun_ = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Protected methods

////////////////////////////////////////////////////////////////////////////////////////////////////
bool BaseController::checkGoalReached(Pose<> currentPose)
{
    double distanceToGoal = PoseMath::euclidean(currentPose, getGoal());
    ROS_DEBUG_STREAM_NAMED("base_controller", "Distance to goal line: " << distanceToGoal);

    // If the distance to the goal line is less than
    // the minimum tolerance, respond right away
    if (distanceToGoal < robot_.physicalMinDistanceToGoal)
    {
        return true;
    }

    // Otherwise, check the intersection of the current pose with
    // the polygon of the goal landing
    bool isIntersecting = PoseMath::intersection(goalLanding_, currentPose);
    ROS_DEBUG_STREAM_NAMED("base_controller", "Inside landing zone: " << isIntersecting);

    return isIntersecting;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void BaseController::executeCommand(Velocity<> command)
{
    executeCommand(command.linear, command.angular);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void BaseController::executeCommand(double linear, double angular)
{
    double finalLinear = BasicMath::saturate<double>(Kv_ * linear,
        robot_.physicalMaxLinearVelocity, -robot_.physicalMaxLinearVelocity);

    double finalAngular = BasicMath::saturate<double>(Kw_ * angular,
        robot_.physicalMaxAngularVelocity, -robot_.physicalMaxAngularVelocity);

    finalLinear = BasicMath::threshold<double>(finalLinear,
        robot_.physicalMinLinearVelocity, 0.0);

    finalAngular = BasicMath::threshold<double>(finalAngular,
        robot_.physicalMinAngularVelocity, 0.0);

    executingCommand_ = Velocity<>(finalLinear, finalAngular);

    ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "base_controller",
        "Executing command: " << executingCommand_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
