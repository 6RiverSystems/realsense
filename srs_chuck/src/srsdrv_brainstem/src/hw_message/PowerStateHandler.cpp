#include <hw_message/PowerStateHandler.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

PowerStateHandler::PowerStateHandler(BrainStemMessageProcessorInterface* processor, ChannelBrainstemPowerState::Interface& publisher) :
	HardwareMessageHandler(processor, BRAIN_STEM_MSG::POWER_STATE),
	publisher_(publisher)
{
	validDescriptors_.insert(BATTERY_DESCRIPTOR::TEMPERATURE);
	validDescriptors_.insert(BATTERY_DESCRIPTOR::VOLTAGE);
	validDescriptors_.insert(BATTERY_DESCRIPTOR::AVERAGE_CURRENT);
	validDescriptors_.insert(BATTERY_DESCRIPTOR::CHARGED_PERCENTAGE);
	validDescriptors_.insert(BATTERY_DESCRIPTOR::AVERAGE_TIME_TO_EMPTY);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void PowerStateHandler::readBatteryDescriptorInfo(HardwareMessage& msg,
	int descriptorIndex, srslib_framework::MsgPowerState& batteryState)
{
	uint8_t id = msg.read<uint8_t>();

	if (validDescriptors_.find(static_cast<BATTERY_DESCRIPTOR>(id)) != validDescriptors_.end())
	{
		for (auto& batterIter : batteryState.batteries)
		{
			srslib_framework::MsgBatteryDescriptor batteryDescriptor;

			batteryDescriptor.id = id;
			batteryDescriptor.value = msg.read<uint16_t>();

			batterIter.descriptors.push_back(batteryDescriptor);
	   }
	}
	else
	{
		throw std::runtime_error("Invalid battery descriptor");
	}
}

void PowerStateHandler::receiveMessage(ros::Time currentTime, HardwareMessage& msg)
{
	PowerStateMsg powerState = msg.read<PowerStateMsg>();

	srslib_framework::MsgPowerState powerStateMsg;

	for (int i = 0; i < powerState.numberOfBatteries; i++)
	{
		srslib_framework::MsgBatteryState batteryState;

		powerStateMsg.batteries.push_back(batteryState);
	}

	for (int i = 0; i < powerState.numberOfBatteryDescriptors; i++)
	{
		readBatteryDescriptorInfo(msg, i, powerStateMsg);
	}

	std::ostringstream stream;
	stream << "Power state: ";

	int i = 1;
	for (const auto& battery : powerStateMsg.batteries)
	{
		stream << "Battery " << i << ": ";

		stream << "[";

		for (const auto& descriptor : battery.descriptors)
		{
			stream << descriptor.id << ": " << descriptor.value << " ";
		}

		stream << "]";

		i++;
	}

	stream << std::endl;

	std::string strData =  stream.str( );

	ROS_INFO_STREAM( strData );

	// Publish the power State
	publisher_.publish(powerStateMsg);
}

} // namespace srs
