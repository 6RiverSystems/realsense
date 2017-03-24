/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherBoolean.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class ChannelBrainstemConnected :
    public PublisherBoolean
{
public:
	ChannelBrainstemConnected() :
		PublisherBoolean(ChuckTopics::driver::BRAINSTEM_STATE_CONNECTED, 1, true)
    {}
};

} // namespace srs
