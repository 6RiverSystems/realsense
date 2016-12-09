#include <srsdrv_brainstem/hw_message/MessageHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

MessageHandler::MessageHandler() :
    HardwareMessageHandler(BRAIN_STEM_MSG::MESSAGE)
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MessageHandler::receiveMessage(ros::Time currentTime, HardwareMessage& msg)
{
	std::string strMessage = msg.readString();

	if( strMessage.find( "<MSG Error" ) != -1 )
	{
		ROS_ERROR_NAMED( "firmware", "Fatal Error: %s", strMessage.c_str( ) );
	}
	else
	{
		ROS_INFO_STREAM_NAMED( "firmware", "Message: <" << strMessage << ">" );
	}
}

} // namespace srs
