/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherUInt32.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelBrainstemButtonPressed :
    public PublisherUInt32
{
public:
	ChannelBrainstemButtonPressed() :
		PublisherUInt32(ChuckTopics::driver::BRAINSTEM_BUTTON_PRESSED)
    {}
};

} // namespace srs
