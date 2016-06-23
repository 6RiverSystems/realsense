#include <srslib_framework/robotics/controller/BaseController.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant initialization
const Velocity<> BaseController::ZERO_VELOCITY = Velocity<>(0.0, 0.0);
const Pose<> BaseController::ZERO_POSE = Pose<>(0.0, 0.0, 0.0);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void BaseController::reset()
{
    canceled_ = false;

    goal_ = ZERO_POSE;
    goalReached_ = false;

    isRobotMoving_ = false;

    executingCommand_ = ZERO_VELOCITY;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void BaseController::step(double dT, Pose<> currentPose, Odometry<> currentOdometry)
{
    // Perform some basic operations before calling
    // the specific controller step
    isRobotMoving_ = !VelocityMath::equal<double>(currentOdometry.velocity, ZERO_VELOCITY);

    // Call the controller
    stepController(dT, currentPose, currentOdometry);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Protected methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void BaseController::executeCommand(Velocity<> command)
{
    executeCommand(command.linear, command.angular);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void BaseController::executeCommand(double linear, double angular)
{
    double finalLinear = BasicMath::saturate<double>(Kv_ * linear,
        robot_.maxLinearVelocity, -robot_.maxLinearVelocity);

    double finalAngular = BasicMath::saturate<double>(Kw_ * angular,
        robot_.maxAngularVelocity, -robot_.maxAngularVelocity);

    finalLinear = BasicMath::threshold<double>(finalLinear,
        robot_.minPhysicalLinearVelocity, 0.0);

    finalAngular = BasicMath::threshold<double>(finalAngular,
        robot_.minPhysicalAngularVelocity, 0.0);

    executingCommand_ = Velocity<>(finalLinear, finalAngular);

    ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "BaseController",
        "Executing command: " << executingCommand_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
