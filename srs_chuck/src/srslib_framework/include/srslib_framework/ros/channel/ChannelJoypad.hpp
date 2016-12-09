/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherJoypadState.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelJoypad :
    public PublisherJoypadState
{
public:
    ChannelJoypad() :
        PublisherJoypadState(ChuckTopics::sensor::JOYPAD_STATE, 50)
    {}
};

} // namespace srs
