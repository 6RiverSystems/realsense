/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherOperationalState.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelBrainstemCmd_ClearMotionState :
    public PublisherOperationalState
{
public:
    ChannelBrainstemCmd_ClearMotionState() :
        PublisherSound(ChuckTopics::driver::BRAINSTEM_CMD_CLEAR_MOTION_STATE)
    {}
};

} // namespace srs
