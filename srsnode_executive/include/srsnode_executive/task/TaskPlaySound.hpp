/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/robotics/device/Sound.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemCmd_Sound.hpp>

#include <srsnode_executive/task/Task.hpp>

namespace srs {

class TaskPlaySound : public Task
{
public:
    static const Sound SOUND_OFF;
    static const Sound SOUND_WARNING;

    TaskPlaySound()
    {
        channelSound_.publish(SOUND_OFF);
    }

    virtual ~TaskPlaySound()
    {}

    void run(ExecutiveContext& context);

private:
    ChannelBrainstemCmd_Sound channelSound_;
};

} // namespace srs
