#include <srsdrv_brainstem/hw_publisher/PowerStatePublisher.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

PowerStatePublisher::PowerStatePublisher(string nameSpace) :
	rosNodeHandle_(nameSpace)
{
	publisher_ = rosNodeHandle_.advertise<srslib_framework::MsgPowerState>(
		ChuckTopics::driver::BRAINSTEM_STATE_POWER, 1, true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void PowerStatePublisher::publishPowerState(srslib_framework::MsgPowerState& powerStateMsg)
{
	publisher_.publish(powerStateMsg);
}

} // namespace srs
