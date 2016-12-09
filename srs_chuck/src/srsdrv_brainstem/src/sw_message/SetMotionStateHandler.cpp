#include <srsdrv_brainstem/sw_message/SetMotionStateHandler.hpp>

#include <srsdrv_brainstem/BrainStemMessageProcessor.h>

#include <ros/ros.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
SetMotionStateHandler::SetMotionStateHandler(BrainStemMessageProcessor* owner) :
    SoftwareMessageHandler(owner)
{
	tapSetMotionState_.attach(this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SetMotionStateHandler::notified(Subscriber<srslib_framework::MsgSetOperationalState>* subject)
{
	TapBrainstemCmd_SetMotionState* tap = static_cast<TapBrainstemCmd_SetMotionState*>(subject);

	srslib_framework::MsgSetOperationalState setOpState = tap->pop();

	std::bitset<8> hardStopSet;
	hardStopSet.set( MOTION_STATUS::FRONT_E_STOP, setOpState.operationalState.frontEStop );
	hardStopSet.set( MOTION_STATUS::HARD_STOP, setOpState.operationalState.hardStop );
	hardStopSet.set( MOTION_STATUS::BACK_E_STOP, setOpState.operationalState.backEStop );
	hardStopSet.set( MOTION_STATUS::WIRELESS_E_STOP, setOpState.operationalState.wirelessEStop );
	hardStopSet.set( MOTION_STATUS::BUMP_SENSOR, setOpState.operationalState.bumpSensor );
	hardStopSet.set( MOTION_STATUS::FREE_SPIN, setOpState.operationalState.pause );

	getOwner()->SetMotionStatus(hardStopSet, setOpState.state);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
