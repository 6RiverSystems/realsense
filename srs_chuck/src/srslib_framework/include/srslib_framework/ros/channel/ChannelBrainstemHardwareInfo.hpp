/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherHardwareInfo.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelBrainstemHardwareInfo :
    public PublisherHardwareInfo
{
public:
	ChannelBrainstemHardwareInfo() :
		PublisherHardwareInfo(ChuckTopics::driver::BRAINSTEM_HARDWARE_INFO)
    {}
};

} // namespace srs
