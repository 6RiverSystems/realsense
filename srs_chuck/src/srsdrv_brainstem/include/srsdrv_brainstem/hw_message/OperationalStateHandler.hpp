/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>
#include <HardwareMessageHandler.hpp>

#include <srslib_framework/ros/channel/ChannelBrainstemOperationalState.hpp>

namespace srs {

class OperationalStateHandler : public HardwareMessageHandler
{
public:

	OperationalStateHandler(ChannelBrainstemOperationalState::Interface& channel);

    virtual ~OperationalStateHandler() {}

    void receiveMessage(ros::Time currentTime, HardwareMessage& msg);

    void setBrainstemTimeout(bool brainstemTimeout);

private:

    HW_MESSAGE_BEGIN(OperationalStateData)
    	uint8_t 	cmd;
    	uint32_t	upTime;
    	uint8_t		motionStatus;
    	uint8_t 	failureStatus;
    HW_MESSAGE_END

	srslib_framework::MsgOperationalState operationalState_;

    ChannelBrainstemOperationalState::Interface& publisher_;
};

} // namespace srs
