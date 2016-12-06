/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include  <memory>

using namespace std;

#include <srsdrv_brainstem/BrainStemMessages.h>
#include <srsdrv_brainstem/hw_message/HardwareMessageHandler.hpp>
#include <srslib_framework/MsgPowerState.h>

namespace srs {

class PowerStateHandler : public HardwareMessageHandler
{
public:

	typedef function<void(srslib_framework::MsgPowerState&)> PowerStateFn;

	PowerStateHandler(PowerStateFn powerStateCallback = [&](srslib_framework::MsgPowerState&) {});

    virtual ~PowerStateHandler() {}

    bool receiveMessage(ros::Time currentTime, HardwareMessage& msg);

private:

    HW_MESSAGE_BEGIN(PowerStateMsg)
        uint8_t cmd;
        uint8_t numberOfBatteries;
        uint8_t numberOfBatteryDescriptors;
    HW_MESSAGE_END

    HW_MESSAGE_BEGIN(BatteryDescriptorMsg)
        uint8_t id;
        uint16_t value;
    HW_MESSAGE_END

	void readBatteryDescriptorInfo(HardwareMessage& msg, int descriptorIndex,
		srslib_framework::MsgPowerState& batteryState);

    void publishPowerState(const srslib_framework::MsgPowerState& powerState);

    PowerStateFn powerStateCallback_;

    set<BATTERY_DESCRIPTOR> validDescriptors_;
};

} // namespace srs
