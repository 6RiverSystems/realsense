/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsdrv_brainstem/sw_message/StartupHandler.hpp>

#include <srsdrv_brainstem/BrainStemMessageProcessorInterface.h>

#include <ros/ros.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
StartupHandler::StartupHandler(BrainStemMessageProcessorInterface* owner) :
    SoftwareMessageHandler(owner)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void StartupHandler::attach()
{
	tapStartup_.reset(new TapBrainstemCmd_Startup());

	tapStartup_->attach(this);
}

void StartupHandler::notified(Subscriber<std_msgs::Bool>* subject)
{
	encodeData(true);
}

void StartupHandler::encodeData(const bool&)
{
	uint8_t cMessage = static_cast<uint8_t>(BRAIN_STEM_CMD::STARTUP);

	getOwner()->sendCommand(reinterpret_cast<char*>(&cMessage), 1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
