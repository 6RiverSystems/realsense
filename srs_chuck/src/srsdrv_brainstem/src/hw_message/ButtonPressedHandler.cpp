#include <srsdrv_brainstem/hw_message/ButtonPressedHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

ButtonPressedHandler::ButtonPressedHandler(ChannelBrainstemButtonPressed channel) :
    HardwareMessageHandler(BRAIN_STEM_MSG::BUTTON_PRESSED),
	channel_(channel)
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void ButtonPressedHandler::receiveMessage(ros::Time currentTime, HardwareMessage& msg)
{
	ButtonPressedData buttonPressedData = msg.read<ButtonPressedData>();

	// TODO: Validate button ID

	ROS_INFO_STREAM("Button Pressed: " << buttonPressedData.buttonId);

	channel_.publish(buttonPressedData.buttonId);
}

} // namespace srs
