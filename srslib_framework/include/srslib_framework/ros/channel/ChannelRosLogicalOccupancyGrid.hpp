/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherRosLogicalOccupancyGrid.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>
#include <srslib_framework/chuck/ChuckTransforms.hpp>

namespace srs {

class ChannelRosLogicalOccupancyGrid :
    public PublisherRosLogicalOccupancyGrid
{
public:
    ChannelRosLogicalOccupancyGrid() :
        PublisherRosLogicalOccupancyGrid(ChuckTopics::internal::MAP_ROS_OCCUPANCY,
            ChuckTransforms::MAP, 1, true)
    {}
};

} // namespace srs