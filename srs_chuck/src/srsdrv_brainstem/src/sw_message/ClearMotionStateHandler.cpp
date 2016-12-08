#include <srsdrv_brainstem/BrainStemMessageProcessor.h>

#include <ros/ros.h>
#include <sw_message/ClearMotionStateHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
ClearMotionStateHandler::ClearMotionStateHandler(BrainStemMessageProcessor* owner) :
    SoftwareMessageHandler(owner)
{
	tapSetMotionState_.attach(this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void ClearMotionStateHandler::notified(Subscriber<srslib_framework::MsgOperationalState>* subject)
{
	TapBrainstemCmd_SetMotionState* tap = static_cast<TapBrainstemCmd_SetMotionState*>(subject);

	srslib_framework::MsgOperationalState operationalState = tap->pop();

	std::bitset<8> hardStopSet;
	hardStopSet.set( MOTION_STATUS::FRONT_E_STOP, operationalState.frontEStop );
	hardStopSet.set( MOTION_STATUS::HARD_STOP, operationalState.hardStop );
	hardStopSet.set( MOTION_STATUS::BACK_E_STOP, operationalState.backEStop );
	hardStopSet.set( MOTION_STATUS::WIRELESS_E_STOP, operationalState.wirelessEStop );
	hardStopSet.set( MOTION_STATUS::BUMP_SENSOR, operationalState.bumpSensor );
	hardStopSet.set( MOTION_STATUS::FREE_SPIN, operationalState.pause );

	getOwner()->SetMotionStatus(hardStopSet, true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
