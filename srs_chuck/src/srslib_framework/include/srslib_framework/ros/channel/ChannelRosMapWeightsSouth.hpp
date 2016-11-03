/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherRosCostGrid.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelRosMapWeightsSouth :
    public PublisherRosCostGrid
{
public:
    ChannelRosMapWeightsSouth() :
        PublisherRosCostGrid(ChuckTopics::internal::MAP_ROS_WEIGHTS_SOUTH, "map", 1, true)
    {}
};

} // namespace srs
