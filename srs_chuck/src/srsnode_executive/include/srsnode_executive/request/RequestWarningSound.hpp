/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/robotics/device/Sound.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemCmd_Sound.hpp>

#include <srsnode_executive/request/Request.hpp>

namespace srs {

class RequestWarningSound : public Request
{
public:
    static const Sound SOUND_OFF;
    static const Sound WARNING_SOUND;

    RequestWarningSound() :
        Request()
    {
        channelSound_.publish(SOUND_OFF);
    }

    void task();

private:
    ChannelBrainstemCmd_Sound channelSound_;
};

} // namespace srs
