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
#include <srsdrv_brainstem/hw_message/HardwareMessageHandler.hpp>
#include <srslib_framework/MsgOperationalState.h>
#include <srslib_framework/ros/channel/ChannelBrainstemOperationalState.hpp>

namespace srs {

class OperationalStateHandler : public HardwareMessageHandler
{
public:

	OperationalStateHandler(ChannelBrainstemOperationalState channel);

    virtual ~OperationalStateHandler() {}

    void receiveMessage(ros::Time currentTime, HardwareMessage& msg);

private:

    HW_MESSAGE_BEGIN(OperationalStateData)
    	uint8_t 	cmd;
    	uint32_t	upTime;
    	uint8_t		motionStatus;
    	uint8_t 	failureStatus;
    HW_MESSAGE_END

    ChannelBrainstemOperationalState channel_;
};

} // namespace srs
