/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherSound.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelBrainstemCmd_Sound :
    public PublisherSound
{
public:
    ChannelBrainstemCmd_Sound() :
        PublisherSound(ChuckTopics::driver::BRAINSTEM_CMD_SOUND)
    {}
};

} // namespace srs
