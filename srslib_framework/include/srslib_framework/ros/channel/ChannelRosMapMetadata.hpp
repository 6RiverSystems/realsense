/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherRosMapMetadata.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class ChannelRosMapMetadata :
    public PublisherRosMapMetadata
{
public:
    ChannelRosMapMetadata() :
        PublisherRosMapMetadata(ChuckTopics::internal::MAP_ROS_METADATA, 1, true)
    {}
};

} // namespace srs
