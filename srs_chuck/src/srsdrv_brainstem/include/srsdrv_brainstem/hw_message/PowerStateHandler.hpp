/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>
#include <hw_message/HardwareMessageHandler.hpp>

#include <srslib_framework/ros/channel/ChannelBrainstemPowerState.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemPowerStateFiltered.hpp>

#include <functional>

namespace srs {

class PowerStateHandler : public HardwareMessageHandler
{
public:

	PowerStateHandler(ChannelBrainstemPowerState::Interface& publisher);

    virtual ~PowerStateHandler() {}

    void receiveMessage(ros::Time currentTime, HardwareMessage& msg);

    void receiveMessage(srslib_framework::MsgPowerState& powerStateMessage);

    void setHook(std::function<void(const srslib_framework::MsgPowerState&)> hook);

private:
    std::string getBatteryDescriptorName(uint8_t id);

    float convertBatteryDescriptorValue(uint8_t id, uint16_t value);

	void readBatteryDescriptorInfo(HardwareMessage& msg, int descriptorIndex,
		srslib_framework::MsgPowerState& batteryState);

    void publishPowerState(const srslib_framework::MsgPowerState& powerState);

    HW_MESSAGE_BEGIN(PowerStateMsg)
        uint8_t cmd;
        uint8_t numberOfBatteries;
        uint8_t numberOfBatteryDescriptors;
    HW_MESSAGE_END

    HW_MESSAGE_BEGIN(BatteryDescriptorData)
        uint8_t id;
        uint16_t value;
    HW_MESSAGE_END

	std::function<void(const srslib_framework::MsgPowerState&)> hook_;

    ChannelBrainstemPowerState::Interface& publisher_;

    std::set<BATTERY_DESCRIPTOR> validDescriptors_;
};

} // namespace srs
