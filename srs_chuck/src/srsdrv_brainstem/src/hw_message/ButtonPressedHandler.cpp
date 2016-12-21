#include <srsdrv_brainstem/hw_message/ButtonPressedHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

ButtonPressedHandler::ButtonPressedHandler(ChannelBrainstemButtonPressed::Interface& publisher) :
    HardwareMessageHandler(BRAIN_STEM_MSG::BUTTON_PRESSED),
	publisher_(publisher)
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void ButtonPressedHandler::receiveMessage(ros::Time currentTime, HardwareMessage& msg)
{
	ButtonPressedData buttonPressedData = msg.read<ButtonPressedData>();

	// TODO (danw): Validate button ID

	ROS_DEBUG_STREAM("Button Pressed: " << (int)buttonPressedData.buttonId);

	publisher_.publish(buttonPressedData.buttonId);
}

} // namespace srs
