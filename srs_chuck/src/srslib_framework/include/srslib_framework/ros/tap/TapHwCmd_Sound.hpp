/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberSound.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class TapHwCmd_Sound :
    public SubscriberSound
{
public:
    TapHwCmd_Sound() :
        SubscriberSound(ChuckTopics::internal::HWCMD_SOUND)
    {}
};

} // namespace srs
