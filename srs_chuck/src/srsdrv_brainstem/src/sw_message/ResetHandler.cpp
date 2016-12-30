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
    SoftwareMessageHandler(owner),
	sentReset_(false)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void ResetHandler::attach()
{
	tapReset_.reset(new TapBrainstemCmd_Reset());

	tapReset_->attach(this);
}

void ResetHandler::notified(Subscriber<std_msgs::Bool>* subject)
{
	TapBrainstemCmd_Reset* tap = static_cast<TapBrainstemCmd_Reset*>(subject);

	encodeData(tap->pop());
}

void ResetHandler::encodeData(const bool& value)
{
	if (!value)
	{
		WatchdogTimeoutData msg = {
			static_cast<uint8_t>(BRAIN_STEM_CMD::FORCE_WATCHDOG_TIMEOUT),
			{ 'p', '0', 'w', 'n' }
		};

		ROS_ERROR("Brainstem driver: Forcing watchdog reset timeout");

		getOwner()->sendCommand( reinterpret_cast<char*>( &msg ), sizeof(msg));

		sentReset_ = true;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
