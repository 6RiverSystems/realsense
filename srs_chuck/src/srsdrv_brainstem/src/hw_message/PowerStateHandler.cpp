#include <hw_message/PowerStateHandler.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>
#include <srslib_framework/robotics/device/PowerState.hpp>

#include <filters/PowerStateFilter.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

PowerStateHandler::PowerStateHandler(ChannelBrainstemPowerState::Interface& publisher) :
	HardwareMessageHandler(BRAIN_STEM_MSG::POWER_STATE),
	publisher_(publisher)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////

float PowerStateHandler::convertBatteryDescriptorValue(BatteryState::Descriptor descriptor, uint16_t value)
{
	switch (descriptor)
	{
		case BatteryState::Descriptor::TEMPERATURE:
			return (float)value / 10.0 - 273.15; // originally in dK
		case BatteryState::Descriptor::VOLTAGE:
			return (float)value / 1000.0;  // originally in mV
		case BatteryState::Descriptor::AVERAGE_CURRENT:
		{
			int16_t signed_val = (int16_t)value;
			return (float)signed_val / 1000.0; // originally in mA
		}
		case BatteryState::Descriptor::INSTANTANEOUS_CURRENT:
		{
			int16_t signed_val = (int16_t)value;
			return (float)signed_val / 1000.0; // originally in mA
		}
		case BatteryState::Descriptor::CHARGED_PERCENTAGE:
			return (float)value / 100.0;  // originally in 0-100
		case BatteryState::Descriptor::AVERAGE_TIME_TO_EMPTY:
			return (float)value; // Time in minutes
		default:
			ROS_WARN("Unknown battery descriptor");
			return (float)value;
	}
}

void PowerStateHandler::readBatteryDescriptorInfo(HardwareMessage& msg,
	int descriptorIndex, PowerState& powerState)
{
	BatteryState::Descriptor descriptor = PowerStateMessageFactory::convertDescriptor(msg.read<uint8_t>());

	for (auto& batterIter : powerState.batteries)
	{
		uint16_t value = msg.read<uint16_t>();
		batterIter.descriptors[descriptor] = convertBatteryDescriptorValue(descriptor, value);
    }
}

void PowerStateHandler::setHook(std::function<void(const PowerState&)> hook)
{
	hook_ = hook;
}

void PowerStateHandler::receiveMessage(ros::Time currentTime, HardwareMessage& msg)
{
	PowerStateMsg msgPowerState = msg.read<PowerStateMsg>();

	PowerState powerState;

	ROS_INFO_STREAM("Power state: " << (int)msgPowerState.numberOfBatteries << std::endl );
	ROS_INFO_STREAM("Power state: " << (int)msgPowerState.numberOfBatteryDescriptors << std::endl );

	for (int i = 0; i < msgPowerState.numberOfBatteries; i++)
	{
		BatteryState batteryState;

		powerState.batteries.push_back(batteryState);
	}

	for (int i = 0; i < msgPowerState.numberOfBatteryDescriptors; i++)
	{
		readBatteryDescriptorInfo(msg, i, powerState);
	}

	ROS_INFO_STREAM_THROTTLE( 60, "Power state: " << powerState << std::endl );

	if (hook_)
	{
		hook_(powerState);
	}

	// Publish the power State
	publisher_.publish(powerState);
}

} // namespace srs
