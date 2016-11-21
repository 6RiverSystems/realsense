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

class StopSound :
    public Task<ExecutiveContext>
{
public:
    StopSound(Condition<ExecutiveContext>* preCondition = nullptr) :
        Task<ExecutiveContext>(preCondition)
    {}

    virtual ~StopSound()
    {}

    TreeNode<ExecutiveContext>::NodeResult execute(ExecutiveContext* context);
private:
    static const Sound SOUND_OFF;

    ChannelBrainstemCmd_Sound channelSound_;
};

} // namespace srs
