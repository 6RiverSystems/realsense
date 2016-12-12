#include "../../include/srsdrv_brainstem/hw_message/LogHandler.hpp"

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

LogHandler::LogHandler(LogCallbackFn logCallback) :
    HardwareMessageHandler(BRAIN_STEM_MSG::LOG),
	logCallback_(logCallback)
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogHandler::receiveMessage(ros::Time currentTime, HardwareMessage& msg)
{
	LogData log = msg.read<LogData>();

	std::string message = msg.readString();

	switch (log.level)
	{
		case 0:
		{
			ROS_DEBUG_NAMED( "firmware", "Firmware: %s", message.c_str( ) );
		}
		break;

		case 1:
		{
			ROS_INFO_NAMED( "firmware", "Firmware: %s", message.c_str( ) );
		}
		break;

		case 2:
		{
			ROS_ERROR_NAMED( "firmware", "Firmware: %s", message.c_str( ) );
		}
		break;
	}

	logCallback_(static_cast<LOG_LEVEL>(log.level), message);
}

} // namespace srs
