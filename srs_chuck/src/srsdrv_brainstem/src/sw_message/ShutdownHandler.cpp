/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsdrv_brainstem/sw_message/ShutdownHandler.hpp>

#include <srsdrv_brainstem/BrainStemMessageProcessorInterface.h>

#include <ros/ros.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////TapBrainstemCmd_Startup
ShutdownHandler::ShutdownHandler(BrainStemMessageProcessorInterface* owner) :
    SoftwareMessageHandler(owner)
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

	getOwner()->sendCommand(reinterpret_cast<char*>(&cMessage), 1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
