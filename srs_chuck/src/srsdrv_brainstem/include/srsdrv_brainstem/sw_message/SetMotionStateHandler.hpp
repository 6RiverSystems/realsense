/*
SetMotionStateHandler.hpp * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/MsgSetOperationalState.h>

#include <srslib_framework/platform/SoftwareMessageHandler.hpp>
#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>
#include <srslib_framework/ros/tap/subscriber/Observer.hpp>
#include <srslib_framework/ros/tap/TapBrainstemCmd_SetMotionState.hpp>

#include "../BrainStemMessages.hpp"

namespace srs {

class BrainStemMessageProcessorInterface;

class SetMotionStateHandler :
    public SoftwareMessageHandler<BrainStemMessageProcessorInterface>,
    public Observer<Subscriber<srslib_framework::MsgSetOperationalState>>
{
public:
    SetMotionStateHandler(BrainStemMessageProcessorInterface* owner);

    virtual ~SetMotionStateHandler() {}

    virtual void attach();

    void notified(Subscriber<srslib_framework::MsgSetOperationalState>* subject);

    void encodeData(const srslib_framework::MsgSetOperationalState& value);

private:

    HW_MESSAGE_BEGIN(OperationalStateData)
    	uint8_t cmd;
		uint8_t motionStatus;
	HW_MESSAGE_END

    std::shared_ptr<TapBrainstemCmd_SetMotionState>	tapSetMotionState_;
};

} // namespace srs
