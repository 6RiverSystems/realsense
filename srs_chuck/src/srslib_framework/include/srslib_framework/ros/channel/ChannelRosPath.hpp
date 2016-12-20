/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherRosPath.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>
#include <srslib_framework/ros/topics/ChuckTransforms.hpp>

namespace srs {

class ChannelRosPath :
    public PublisherRosPath
{
public:
    ChannelRosPath() :
        PublisherRosPath(ChuckTopics::internal::MAP_ROS_PATH,
            ChuckTransforms::MAP, 1, true)
    {}
};

} // namespace srs
