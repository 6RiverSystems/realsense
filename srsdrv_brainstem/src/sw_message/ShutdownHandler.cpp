/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <BrainStemMessageProcessorInterface.hpp>

#include <sw_message/ShutdownHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////TapBrainstemCmd_Startup
ShutdownHandler::ShutdownHandler(BrainStemMessageProcessorInterface* owner) :
	SoftwareMessage(owner)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void ShutdownHandler::attach()
{
	tapShutdown_.reset(new TapBrainstemCmd_Shutdown());

	tapShutdown_->attach(this);
}

void ShutdownHandler::notified(Subscriber<std_msgs::Bool>* subject)
{
	encodeData(true);
}

void ShutdownHandler::encodeData(const bool& shutdown)
{
	uint8_t cMessage = static_cast<uint8_t>(BRAIN_STEM_CMD::SHUTDOWN);

	ROS_INFO( "Brain => Brainstem: SHUTDOWN");

	sendCommand(reinterpret_cast<char*>(&cMessage), 1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
