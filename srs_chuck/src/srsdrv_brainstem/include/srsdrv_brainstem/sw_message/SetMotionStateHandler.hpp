/*
SetMotionStateHandler.hpp * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>

#include <srslib_framework/platform/SoftwareMessageHandler.hpp>
#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>
#include <srslib_framework/platform/observer/Observer.hpp>
#include <srslib_framework/ros/tap/TapBrainstemCmd_SetMotionState.hpp>

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

    void syncState();

private:

    HW_MESSAGE_BEGIN(OperationalStateData)
    	uint8_t cmd;
		uint8_t motionStatus;
	HW_MESSAGE_END

	srslib_framework::MsgSetOperationalState setOpState_;

    std::shared_ptr<TapBrainstemCmd_SetMotionState>	tapSetMotionState_;
};

} // namespace srs
