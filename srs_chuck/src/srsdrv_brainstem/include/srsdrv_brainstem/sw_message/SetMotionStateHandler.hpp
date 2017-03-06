/*
SetMotionStateHandler.hpp * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>
#include <sw_message/SoftwareMessage.hpp>

#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>
#include <srslib_framework/platform/observer/Observer.hpp>
#include <srslib_framework/ros/tap/TapBrainstemCmd_SetMotionState.hpp>

namespace srs {

class BrainStemMessageProcessorInterface;

class SetMotionStateHandler :
	public SoftwareMessage,
    public Observer<Subscriber<srslib_framework::MsgSetOperationalState>>
{
public:
    SetMotionStateHandler(BrainStemMessageProcessorInterface* owner);

    virtual ~SetMotionStateHandler() {}

    virtual void attach();

    void notified(Subscriber<srslib_framework::MsgSetOperationalState>* subject);

    void encodeData(const srslib_framework::MsgSetOperationalState& value);

    void syncState();

    void setOperationalState(uint8_t opState, bool setValues);

private:

    HW_MESSAGE_BEGIN(OperationalStateData)
    	uint8_t cmd;
		uint8_t motionStatus;
	HW_MESSAGE_END

	uint8_t operationalState_;

    std::shared_ptr<TapBrainstemCmd_SetMotionState>	tapSetMotionState_;
};

} // namespace srs