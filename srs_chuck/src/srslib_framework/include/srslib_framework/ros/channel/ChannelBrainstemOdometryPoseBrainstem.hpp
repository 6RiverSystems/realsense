/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherOdometryPose.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class ChannelBrainstemOdometryPoseBrainstem :
    public PublisherOdometryPose
{
public:

	ChannelBrainstemOdometryPoseBrainstem() :
		PublisherOdometryPose(ChuckTopics::sensor::ODOMETRY_POSE_BRAINSTEM)
    {}
};

} // namespace srs