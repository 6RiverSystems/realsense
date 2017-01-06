/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherOdometryPose.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelBrainstemOdometryPose :
    public PublisherOdometryPose
{
public:

	ChannelBrainstemOdometryPose() :
		PublisherOdometryPose(ChuckTopics::sensor::ODOMETRY_POSE_RAW)
    {}
};

} // namespace srs
