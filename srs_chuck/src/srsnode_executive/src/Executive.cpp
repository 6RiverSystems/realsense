#include <srsnode_executive/Executive.hpp>

#include <srslib_framework/math/VelocityMath.hpp>
#include <srslib_framework/ros/topics/ChuckConfig.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Executive::Executive(string name, int argc, char** argv) :
    RosUnit(name, argc, argv, REFRESH_RATE_HZ),
    commandLineHandler_(this)
{
    context_.robotPose = Pose<>::INVALID;
    context_.mapStack = nullptr;
    context_.isRobotMoving = false;
    context_.isRobotPaused = true;

    readConfigurationParameters();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::execute()
{
    updateContext();

    taskDetectLabeledAreas_.run(context_);
    taskSetMaxVelocity_.run(context_);
    taskPlaySound_.run(context_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::initialize()
{
    // Nothing to initialize
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::updateContext()
{
    context_.robotPose = tapRobotPose_.pop();
    context_.commandedVelocity = tapCommandedVelocity_.pop();

    // Make sure that the neither the logical not the occupancy maps
    // have been re-published. In case, destroy what we have and
    // ask for a new stack
    if (tapMapStack_.newDataAvailable())
    {
        context_.mapStack = tapMapStack_.pop();
    }

    // Determine if the robot is currently moving
    context_.isRobotMoving = !VelocityMath::equal(context_.commandedVelocity, Velocity<>::ZERO);

    // The Pause state is handled by one of the commands
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::readConfigurationParameters()
{
    getGlobalParameter(ChuckConfig::Entities::LOCAL_PLANNER, ChuckConfig::Parameters::MAX_VELOCITY,
        context_.maxVelocity, 1.0f);

    taskSetMaxVelocity_.setDefaultMaxVelocity(context_.maxVelocity);
}

} // namespace srs
