/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <BrainStemMessageProcessorInterface.hpp>

#include <sw_message/PingHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
PingHandler::PingHandler(BrainStemMessageProcessorInterface* owner) :
    SoftwareMessageHandler(owner)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PingHandler::attach()
{
	tapPing_.reset(new TapBrainstemCmd_Ping());

	tapPing_->attach(this);
}

void PingHandler::notified(Subscriber<std_msgs::Bool>*)
{
	encodeData(true);
}

void PingHandler::encodeData(const bool& value)
{
	uint8_t cMessage = static_cast<uint8_t>(BRAIN_STEM_CMD::PING);

	ROS_INFO_NAMED("ping", "Brain => Brainstem: Ping");

	getOwner()->sendCommand(reinterpret_cast<char*>(&cMessage), 1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
