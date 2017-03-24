/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherPowerStateFiltered.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class ChannelBrainstemPowerStateFiltered :
    public PublisherPowerStateFiltered
{
public:
	ChannelBrainstemPowerStateFiltered() :
		PublisherPowerStateFiltered(ChuckTopics::driver::BRAINSTEM_STATE_POWER_FILTERED, 1, true)
    {}
};

} // namespace srs
