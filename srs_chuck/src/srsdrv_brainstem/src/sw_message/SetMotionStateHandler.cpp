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
    SoftwareMessageHandler(owner)
{
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
	setOpState_ = setOpState;

	std::bitset<8> motionStateSet;
	motionStateSet.set( MOTION_STATUS::FRONT_E_STOP, setOpState.operationalState.frontEStop );
	motionStateSet.set( MOTION_STATUS::BACK_E_STOP, setOpState.operationalState.backEStop );
	motionStateSet.set( MOTION_STATUS::HARD_STOP, setOpState.operationalState.hardStop );
	motionStateSet.set( MOTION_STATUS::WIRELESS_E_STOP, setOpState.operationalState.wirelessEStop );
	motionStateSet.set( MOTION_STATUS::BUMP_SENSOR, setOpState.operationalState.bumpSensor );
	motionStateSet.set( MOTION_STATUS::FREE_SPIN, setOpState.operationalState.pause );

	std::string strMotionStatus;
	strMotionStatus += setOpState.operationalState.frontEStop ? "frontEStop, " : "";
	strMotionStatus += setOpState.operationalState.backEStop ? "backEStop, " : "";
	strMotionStatus += setOpState.operationalState.wirelessEStop ? "wirelessEStop, " : "";
	strMotionStatus += setOpState.operationalState.bumpSensor ? "bumpSensor, " : "";
	strMotionStatus += setOpState.operationalState.pause ? "free-spin, " : "";
	strMotionStatus += setOpState.operationalState.hardStop ? "hardStop, " : "";

	BRAIN_STEM_CMD command = setOpState.state ? BRAIN_STEM_CMD::SET_MOTION_STATUS : BRAIN_STEM_CMD::CLEAR_MOTION_STATUS;

	OperationalStateData msg = {
        static_cast<uint8_t>(command),
        static_cast<uint8_t>(motionStateSet.to_ulong())
    };

	ROS_INFO( "Brain => Brainstem: %s: %s",
	    command == BRAIN_STEM_CMD::SET_MOTION_STATUS ? "SET_MOTION_STATUS" : "CLEAR_MOTION_STATUS",
	    strMotionStatus.c_str( ) );

	getOwner()->sendCommand( reinterpret_cast<char*>( &msg ), sizeof(msg));
}

void SetMotionStateHandler::syncState()
{
	encodeData(setOpState_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
