/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <functional>

using namespace std;

#include <srsdrv_brainstem/BrainStemMessages.h>
#include <srsdrv_brainstem/HardwareMessageHandler.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemButtonPressed.hpp>

namespace srs {

class ButtonPressedHandler : public HardwareMessageHandler
{
public:

    ButtonPressedHandler(ChannelBrainstemButtonPressed::Interface& publisher);

    virtual ~ButtonPressedHandler() {}

    void receiveMessage(ros::Time currentTime, HardwareMessage& msg);

private:
    HW_MESSAGE_BEGIN(ButtonPressedData)
		uint8_t cmd;
    	uint8_t buttonId;
    HW_MESSAGE_END

	ChannelBrainstemButtonPressed::Interface& publisher_;
};

} // namespace srs
