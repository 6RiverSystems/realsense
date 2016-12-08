/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherSensorFrame.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelBrainstemSensorFrame :
    public PublisherSensorFrame
{
public:
	ChannelBrainstemSensorFrame() :
		PublisherSensorFrame(ChuckTopics::driver::BRAINSTEM_SENSOR_FRAME)
    {}
};

} // namespace srs
