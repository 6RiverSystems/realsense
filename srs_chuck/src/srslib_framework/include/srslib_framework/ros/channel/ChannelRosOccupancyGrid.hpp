/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherRosOccupancyGrid.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelRosOccupancyGrid :
    public PublisherRosOccupancyGrid
{
public:
    ChannelRosOccupancyGrid() :
        PublisherRosOccupancyGrid(ChuckTopics::internal::MAP_ROS_OCCUPANCY, 1, true)
    {}
};

} // namespace srs
