/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <BrainStemMessageProcessorInterface.hpp>

#include <sw_message/ResetHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
ResetHandler::ResetHandler(BrainStemMessageProcessorInterface* owner) :
    SoftwareMessageHandler(owner)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void ResetHandler::attach()
{
	tapReset_.reset(new TapBrainstemCmd_Ping());

	tapReset_->attach(this);
}

void ResetHandler::notified(Subscriber<std_msgs::Bool>*)
{
	encodeData(true);
}

void ResetHandler::encodeData(const bool& value)
{
	WatchdogTimeoutData msg = {
		static_cast<uint8_t>(BRAIN_STEM_CMD::FORCE_WATCHDOG_TIMEOUT),
		{ 'p', '0', 'w', 'n' }
	};

	ROS_ERROR("Brainstem driver: Forcing watchdog reset timeout");

	getOwner()->sendCommand( reinterpret_cast<char*>( &msg ), sizeof(msg), true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
