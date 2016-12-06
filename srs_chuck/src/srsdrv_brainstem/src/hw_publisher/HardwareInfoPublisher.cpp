#include <srsdrv_brainstem/hw_publisher/HardwareInfoPublisher.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

HardwareInfoPublisher::HardwareInfoPublisher(string nameSpace) :
	rosNodeHandle_(nameSpace)
{
	publisher_ = rosNodeHandle_.advertise<srslib_framework::MsgHardwareInfo>(
		ChuckTopics::driver::BRAINSTEM_HARDWARE_INFO, 1, true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void HardwareInfoPublisher::publishHardwareInfo(srslib_framework::MsgHardwareInfo& hardwareInfoMsg)
{
	publisher_.publish(hardwareInfoMsg);
}

} // namespace srs
