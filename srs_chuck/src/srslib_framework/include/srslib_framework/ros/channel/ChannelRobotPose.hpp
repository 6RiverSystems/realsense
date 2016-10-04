/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherPose.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelRobotPose :
    public PublisherPose
{
public:
    ChannelRobotPose() :
        PublisherPose(ChuckTopics::internal::ROBOT_POSE)
    {}
};

} // namespace srs
