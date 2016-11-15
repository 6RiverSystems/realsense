/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherBoolean.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelHwCmd_Honk :
    public PublisherBoolean
{
public:
    ChannelHwCmd_Honk() :
        PublisherBoolean(ChuckTopics::internal::HWCMD_HONK)
    {}
};

} // namespace srs
