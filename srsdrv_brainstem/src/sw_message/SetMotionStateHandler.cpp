/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <bitset>

#include <BrainStemMessageProcessorInterface.hpp>

#include <sw_message/SetMotionStateHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
SetMotionStateHandler::SetMotionStateHandler(BrainStemMessageProcessorInterface* owner) :
	SoftwareMessage(owner),
	operationalState_(0)
{
//	std::bitset<8> operationalStateChange;
//	operationalStateChange.set(MOTION_STATUS::FREE_SPIN, true);
//
//	operationalState_ = (uint8_t)operationalStateChange.to_ulong();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SetMotionStateHandler::attach()
{
	tapSetMotionState_.reset(new TapBrainstemCmd_SetMotionState());

	tapSetMotionState_->attach(this);
}

void SetMotionStateHandler::notified(Subscriber<srslib_framework::MsgSetOperationalState>* subject)
{
	TapBrainstemCmd_SetMotionState* tap = static_cast<TapBrainstemCmd_SetMotionState*>(subject);

	srslib_framework::MsgSetOperationalState setOpState = tap->pop();

	encodeData(setOpState);
}

void SetMotionStateHandler::encodeData(const srslib_framework::MsgSetOperationalState& setOpState)
{
	std::bitset<8> operationalStateChange;
	operationalStateChange.set( MOTION_STATUS::FRONT_E_STOP, setOpState.operationalState.frontEStop );
	operationalStateChange.set( MOTION_STATUS::BACK_E_STOP, setOpState.operationalState.backEStop );
	operationalStateChange.set( MOTION_STATUS::HARD_STOP, setOpState.operationalState.hardStop );
	operationalStateChange.set( MOTION_STATUS::WIRELESS_E_STOP, setOpState.operationalState.wirelessEStop );
	operationalStateChange.set( MOTION_STATUS::BUMP_SENSOR, setOpState.operationalState.bumpSensor );
	operationalStateChange.set( MOTION_STATUS::FREE_SPIN, setOpState.operationalState.pause );

	uint8_t operationalStateMask = operationalStateChange.to_ulong();

	// Update our operational state (clear or set states)
	if (setOpState.state)
	{
		operationalState_ |= operationalStateMask;
	}
	else
	{
		operationalState_ &= ~operationalStateMask;
	}

	setOperationalState(operationalStateMask, setOpState.state);
}

void SetMotionStateHandler::syncState()
{
	setOperationalState(operationalState_, true);
	setOperationalState((uint8_t)~operationalState_, false);
}

void SetMotionStateHandler::setOperationalState(uint8_t opState, bool setValues)
{
	std::bitset<8> operationalState(opState);

	std::string strMotionStatus;
	strMotionStatus += operationalState.test(MOTION_STATUS::FRONT_E_STOP) ? "frontEStop, " : "";
	strMotionStatus += operationalState.test(MOTION_STATUS::BACK_E_STOP) ? "backEStop, " : "";
	strMotionStatus += operationalState.test(MOTION_STATUS::WIRELESS_E_STOP) ? "wirelessEStop, " : "";
	strMotionStatus += operationalState.test(MOTION_STATUS::BUMP_SENSOR) ? "bumpSensor, " : "";
	strMotionStatus += operationalState.test(MOTION_STATUS::FREE_SPIN) ? "pause, " : "";
	strMotionStatus += operationalState.test(MOTION_STATUS::HARD_STOP) ? "hardStop" : "";

	BRAIN_STEM_CMD command = setValues ? BRAIN_STEM_CMD::SET_MOTION_STATUS : BRAIN_STEM_CMD::CLEAR_MOTION_STATUS;

	ROS_INFO( "Brain => Brainstem: %s: %s",
	    command == BRAIN_STEM_CMD::SET_MOTION_STATUS ? "SET_MOTION_STATUS" : "CLEAR_MOTION_STATUS",
	    strMotionStatus.c_str( ) );

	OperationalStateData msg = {
        static_cast<uint8_t>(command),
        static_cast<uint8_t>(opState)
    };

	sendCommand(reinterpret_cast<char*>( &msg ), sizeof(msg));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
