/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/MsgPowerState.h>

#include <srslib_framework/robotics/device/PowerState.hpp>

namespace srs {

struct PowerStateMessageFactory
{
	enum class BATTERY_DESCRIPTOR : uint8_t
	{
		TEMPERATURE				= 0x08,
		VOLTAGE					= 0x9,
		AVERAGE_CURRENT			= 0xB,
		INSTANTANEOUS_CURRENT	= 0xA,
		CHARGED_PERCENTAGE		= 0xD,
		AVERAGE_TIME_TO_EMPTY	= 0x12,
		INVALID					= 0xFF
	};

    /**
     * @brief Convert BatteryState type into a MsgBatteryState.
     *
     * @param batteryState BatteryState to convert
     *
     * @return newly generated message
     */
	static BatteryState::Descriptor convertDescriptor(uint32_t msgDescriptor)
	{
		BatteryState::Descriptor batteryDescriptor = BatteryState::Descriptor::INVALID;

		switch(static_cast<BATTERY_DESCRIPTOR>(msgDescriptor))
		{
			case BATTERY_DESCRIPTOR::TEMPERATURE:
			{
				batteryDescriptor = BatteryState::Descriptor::TEMPERATURE;
			}
			break;

			case BATTERY_DESCRIPTOR::VOLTAGE:
			{
				batteryDescriptor = BatteryState::Descriptor::VOLTAGE;
			}
			break;

			case BATTERY_DESCRIPTOR::AVERAGE_CURRENT:
			{
				batteryDescriptor = BatteryState::Descriptor::AVERAGE_CURRENT;
			}
			break;

			case BATTERY_DESCRIPTOR::INSTANTANEOUS_CURRENT:
			{
				batteryDescriptor = BatteryState::Descriptor::INSTANTANEOUS_CURRENT;
			}
			break;

			case BATTERY_DESCRIPTOR::CHARGED_PERCENTAGE:
			{
				batteryDescriptor = BatteryState::Descriptor::CHARGED_PERCENTAGE;
			}
			break;

			case BATTERY_DESCRIPTOR::AVERAGE_TIME_TO_EMPTY:
			{
				batteryDescriptor = BatteryState::Descriptor::AVERAGE_TIME_TO_EMPTY;
			}
			break;
		}

		return batteryDescriptor;
	}

	static BATTERY_DESCRIPTOR convertDescriptor(BatteryState::Descriptor descriptor)
	{
		BATTERY_DESCRIPTOR msgDescriptor = BATTERY_DESCRIPTOR::INVALID;

		switch(descriptor)
		{
			case BatteryState::Descriptor::TEMPERATURE:
			{
				msgDescriptor = BATTERY_DESCRIPTOR::TEMPERATURE;
			}
			break;

			case BatteryState::Descriptor::VOLTAGE:
			{
				msgDescriptor = BATTERY_DESCRIPTOR::VOLTAGE;
			}
			break;

			case BatteryState::Descriptor::AVERAGE_CURRENT:
			{
				msgDescriptor = BATTERY_DESCRIPTOR::AVERAGE_CURRENT;
			}
			break;

			case BatteryState::Descriptor::INSTANTANEOUS_CURRENT:
			{
				msgDescriptor = BATTERY_DESCRIPTOR::INSTANTANEOUS_CURRENT;
			}
			break;

			case BatteryState::Descriptor::CHARGED_PERCENTAGE:
			{
				msgDescriptor = BATTERY_DESCRIPTOR::CHARGED_PERCENTAGE;
			}
			break;

			case BatteryState::Descriptor::AVERAGE_TIME_TO_EMPTY:
			{
				msgDescriptor = BATTERY_DESCRIPTOR::AVERAGE_TIME_TO_EMPTY;
			}
			break;
		}

		return msgDescriptor;
	}

    /**
     * @brief Convert BatteryState type into a MsgBatteryState.
     *
     * @param batteryState BatteryState to convert
     *
     * @return newly generated message
     */
    static srslib_framework::MsgBatteryState robotBatteryState2Msg(const BatteryState& batteryState)
    {
        srslib_framework::MsgBatteryState msgBatteryState;

    	for(auto descriptor : batteryState.descriptors)
    	{
    		srslib_framework::MsgBatteryDescriptor msgDescriptor;
    		msgDescriptor.id = static_cast<uint8_t>(convertDescriptor(descriptor.first));
    		msgDescriptor.value = descriptor.second;

    		msgBatteryState.descriptors.push_back(msgDescriptor);
    	}

        return msgBatteryState;
    }

    /**
     * @brief Convert a MsgBatteryState type into a BatteryState.
     *
     * @param msg Message to convert
     *
     * @return newly generated BatteryState
     */
    static BatteryState msg2BatteryState(const srslib_framework::MsgBatteryState& msg)
    {
    	BatteryState batterState;

    	for(auto msgDescriptor : msg.descriptors)
    	{
    		batterState.descriptors[convertDescriptor(msgDescriptor.id)] = msgDescriptor.value;
    	}

        return batterState;
    }

    /**
     * @brief Convert a MsgBatteryStateConstPtr type into a BatteryState.
     *
     * @param msg Message to convert
     *
     * @return newly generated BatteryState
     */
    static BatteryState msg2BatteryState(srslib_framework::MsgBatteryState::ConstPtr msg)
    {
        return msg2BatteryState(*msg);
    }

    /**
     * @brief Convert a PowerState type into a MsgPowerState.
     *
     * @param powerState PowerState to convert
     *
     * @return newly generated message
     */
    static srslib_framework::MsgPowerState powerState2Msg(const PowerState& powerState)
    {
        srslib_framework::MsgPowerState msgPowerState;

		for(auto battery : powerState.batteries)
		{
			msgPowerState.batteries.push_back(robotBatteryState2Msg(battery));
		}

        return msgPowerState;
    }

    /**
     * @brief Convert a MsgOperationalState type into a RobotState.
     *
     * @param msg Message to convert
     *
     * @return newly generated PowerState
     */
    static PowerState msg2PowerState(const srslib_framework::MsgPowerState& msg)
    {
    	PowerState powerState;

    	for(auto msgBattery: msg.batteries)
    	{
    		powerState.batteries.push_back(msg2BatteryState(msgBattery));
    	}

        return powerState;
    }

    /**
     * @brief Convert a MsgPowerStateConstPtr type into a PowerState.
     *
     * @param msg Message to convert
     *
     * @return newly generated PowerState
     */
    static PowerState msg2PowerState(srslib_framework::MsgPowerState::ConstPtr msg)
    {
        return msg2PowerState(*msg);
    }
};

} // namespace srs
