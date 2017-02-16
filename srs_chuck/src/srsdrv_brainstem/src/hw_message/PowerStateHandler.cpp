#include <hw_message/PowerStateHandler.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

#include <filters/PowerStateFilter.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

PowerStateHandler::PowerStateHandler(ChannelBrainstemPowerState::Interface& publisher) :
	HardwareMessageHandler(BRAIN_STEM_MSG::POWER_STATE),
	publisher_(publisher)
{
	validDescriptors_.insert(BATTERY_DESCRIPTOR::TEMPERATURE);
	validDescriptors_.insert(BATTERY_DESCRIPTOR::VOLTAGE);
	validDescriptors_.insert(BATTERY_DESCRIPTOR::AVERAGE_CURRENT);
	validDescriptors_.insert(BATTERY_DESCRIPTOR::INSTANTANEOUS_CURRENT);
	validDescriptors_.insert(BATTERY_DESCRIPTOR::CHARGED_PERCENTAGE);
	validDescriptors_.insert(BATTERY_DESCRIPTOR::AVERAGE_TIME_TO_EMPTY);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::string PowerStateHandler::getBatteryDescriptorName(uint8_t id)
{
	BATTERY_DESCRIPTOR bd = static_cast<BATTERY_DESCRIPTOR>(id);
	if (validDescriptors_.find(bd) != validDescriptors_.end())
	{
	    switch (bd)
	    {
	    	case BATTERY_DESCRIPTOR::TEMPERATURE:
	    		return "Temperature (deg C)";
	    	case BATTERY_DESCRIPTOR::VOLTAGE:
	    		return "Voltage (V)";
    		case BATTERY_DESCRIPTOR::AVERAGE_CURRENT:
    			return "Average current (A)";
    		case BATTERY_DESCRIPTOR::INSTANTANEOUS_CURRENT:
    			return "Instantaneous current (A)";
			case BATTERY_DESCRIPTOR::CHARGED_PERCENTAGE:
				return "Percent charged (0-1)";
			case BATTERY_DESCRIPTOR::AVERAGE_TIME_TO_EMPTY:
				return "Average time to empty (min)";
			default:
			{
				ROS_WARN("Unknown battery descriptor");
				std::ostringstream stream;
				stream << std::hex << id;
				return stream.str();
			}
	    }

	}
	else
	{
		throw std::runtime_error("Invalid battery descriptor");
	}
}

float PowerStateHandler::convertBatteryDescriptorValue(uint8_t id, uint16_t value)
{
	BATTERY_DESCRIPTOR bd = static_cast<BATTERY_DESCRIPTOR>(id);
	if (validDescriptors_.find(bd) != validDescriptors_.end())
	{
	    switch (bd)
	    {
	    	case BATTERY_DESCRIPTOR::TEMPERATURE:
	    		return (float)value / 10.0 - 273.15; // originally in dK
	    	case BATTERY_DESCRIPTOR::VOLTAGE:
	    		return (float)value / 1000.0;  // originally in mV
    		case BATTERY_DESCRIPTOR::AVERAGE_CURRENT:
    		{
    			int16_t signed_val = (int16_t)value;
    			return (float)signed_val / 1000.0; // originally in mA
    		}
    		case BATTERY_DESCRIPTOR::INSTANTANEOUS_CURRENT:
    		{
    			int16_t signed_val = (int16_t)value;
    			return (float)signed_val / 1000.0; // originally in mA
    		}
			case BATTERY_DESCRIPTOR::CHARGED_PERCENTAGE:
				return (float)value / 100.0;  // originally in 0-100
			case BATTERY_DESCRIPTOR::AVERAGE_TIME_TO_EMPTY:
				return (float)value; // Time in minutes
			default:
				ROS_WARN("Unknown battery descriptor");
				return (float)value;
	    }

	}
	else
	{
		throw std::runtime_error("Invalid battery descriptor");
	}
}

void PowerStateHandler::readBatteryDescriptorInfo(HardwareMessage& msg,
	int descriptorIndex, srslib_framework::MsgPowerState& batteryState)
{
	uint8_t id = msg.read<uint8_t>();

	std::string descriptor_name = getBatteryDescriptorName(id);

	for (auto& batterIter : batteryState.batteries)
	{
		srslib_framework::MsgBatteryDescriptor batteryDescriptor;
		batteryDescriptor.id = id;
		batteryDescriptor.value = convertBatteryDescriptorValue(id, msg.read<uint16_t>());

		batterIter.descriptors.push_back(batteryDescriptor);
    }
}

void PowerStateHandler::setHook(std::function<void(const srslib_framework::MsgPowerState&)> hook)
{
	hook_ = hook;
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
	stream << "Power state: " << std::endl;

	int i = 1;
	for (const auto& battery : powerStateMsg.batteries)
	{
		stream << "  Battery " << i << ":" << std::endl;

		for (const auto& descriptor : battery.descriptors)
		{
			stream << "    -" << getBatteryDescriptorName(descriptor.id) << ": " << descriptor.value << std::endl;
		}

		i++;
	}

	std::string strData =  stream.str( );

	ROS_INFO_STREAM_THROTTLE( 60, strData );

	if (hook_)
	{
		hook_(powerStateMsg);
	}

	// Publish the power State
	publisher_.publish(powerStateMsg);
}

} // namespace srs
