#include <srsdrv_brainstem/hw_message/ButtonPressedHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

ButtonPressedHandler::ButtonPressedHandler(BrainStemMessageProcessorInterface* processor, ChannelBrainstemButtonPressed::Interface& publisher) :
    HardwareMessageHandler(processor, BRAIN_STEM_MSG::BUTTON_PRESSED),
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
