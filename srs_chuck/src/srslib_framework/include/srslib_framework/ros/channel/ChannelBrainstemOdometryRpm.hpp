/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherOdometryRpm.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class ChannelBrainstemOdometryRpm :
    public PublisherOdometryRpm
{
public:

	ChannelBrainstemOdometryRpm() :
		PublisherOdometryRpm(ChuckTopics::driver::BRAINSTEM_ODOMETRY_RPM)
    {}
};

} // namespace srs
