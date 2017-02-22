/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherMapStack.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class ChannelMapStack :
    public PublisherMapStack
{
public:
    ChannelMapStack() :
        PublisherMapStack(ChuckTopics::internal::MAP_STACK, 1, true)
    {}
};

} // namespace srs
