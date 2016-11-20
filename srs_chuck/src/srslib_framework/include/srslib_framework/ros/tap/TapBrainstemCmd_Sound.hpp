/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberSound.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class TapBrainstemCmd_Sound :
    public SubscriberSound
{
public:
    TapBrainstemCmd_Sound() :
        SubscriberSound(ChuckTopics::driver::BRAINSTEM_CMD_SOUND)
    {}
};

} // namespace srs
