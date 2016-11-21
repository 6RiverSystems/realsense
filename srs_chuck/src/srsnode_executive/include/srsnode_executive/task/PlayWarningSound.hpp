/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/behavior/behavior_tree/TreeNode.hpp>
#include <srslib_framework/behavior/behavior_tree/Task.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemCmd_Sound.hpp>

#include <srsnode_executive/ExecutiveContext.hpp>

namespace srs {

class PlayWarningSound :
    public Task<ExecutiveContext>
{
public:
    PlayWarningSound(Condition<ExecutiveContext>* preCondition = nullptr) :
        Task<ExecutiveContext>(preCondition)
    {}

    virtual ~PlayWarningSound()
    {}

    TreeNode<ExecutiveContext>::NodeResult execute(ExecutiveContext* context);

private:
    static const Sound WARNING_SOUND;

    ChannelBrainstemCmd_Sound channelSound_;
};

} // namespace srs
