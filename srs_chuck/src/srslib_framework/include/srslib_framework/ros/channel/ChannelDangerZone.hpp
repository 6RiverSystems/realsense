/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherPolygonStamped.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelDangerZone :
    public PublisherPolygonStamped
{
public:
    ChannelDangerZone() :
        PublisherPolygonStamped(ChuckTopics::internal::DANGER_ZONE)
    {}
};

} // namespace srs
