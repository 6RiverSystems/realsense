/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherSound.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelHwCmd_Sound :
    public PublisherSound
{
public:
    ChannelHwCmd_Sound() :
        PublisherSound(ChuckTopics::internal::HWCMD_SOUND)
    {}
};

} // namespace srs
