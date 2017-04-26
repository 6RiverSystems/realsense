/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherPowerState.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class ChannelBrainstemPowerState :
    public PublisherPowerState
{
public:
	ChannelBrainstemPowerState() :
    	PublisherPowerState(ChuckTopics::driver::BRAINSTEM_STATE_POWER_RAW, 1, true)
    {}
};

} // namespace srs
