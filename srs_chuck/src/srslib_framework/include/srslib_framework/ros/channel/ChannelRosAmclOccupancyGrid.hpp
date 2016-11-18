/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherRosAmclOccupancyGrid.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelRosAmclOccupancyGrid :
    public PublisherRosAmclOccupancyGrid
{
public:
    ChannelRosAmclOccupancyGrid() :
        PublisherRosAmclOccupancyGrid(ChuckTopics::internal::MAP_ROS_AMCL_OCCUPANCY, "map", 1, true)
    {}
};

} // namespace srs
