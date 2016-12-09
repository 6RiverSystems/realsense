/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherOperationalState.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelBrainstemOperationalState :
    public PublisherOperationalState
{
public:
	ChannelBrainstemOperationalState() :
		PublisherOperationalState(ChuckTopics::driver::BRAINSTEM_OPERATIONAL_STATE, 1, true)
    {}
};

} // namespace srs
