/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherPoseStamped.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class ChannelMoveBaseCmd_GotoGoal :
    public PublisherPoseStamped
{
public:
    ChannelMoveBaseCmd_GotoGoal() :
        PublisherPoseStamped(ChuckTopics::node::MOVE_BASE_CMD_GOTO_GOAL, 1)
    {}
};

} // namespace srs
