#include "../../include/srsdrv_brainstem/hw_message/LogHandler.hpp"

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

LogHandler::LogHandler() :
    HardwareMessageHandler(BRAIN_STEM_MSG::LOG)
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
}

} // namespace srs
