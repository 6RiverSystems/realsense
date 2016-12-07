/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherPolygonStamped.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelFailedLaserScan :
    public PublisherPolygonStamped
{
public:
    ChannelFailedLaserScan() :
        PublisherPolygonStamped(ChuckTopics::internal::DEBUG_DANGER_ZONE_LASER_SCAN)
    {}
};

} // namespace srs
