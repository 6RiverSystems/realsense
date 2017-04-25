/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherPolygonStamped.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class ChannelFailedDangerZone :
    public PublisherPolygonStamped
{
public:
    ChannelFailedDangerZone() :
        PublisherPolygonStamped(ChuckTopics::internal::DEBUG_DANGER_ZONE)
    {}
};

} // namespace srs
