/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberUpdateBodyLights.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class TapBrainstemCmd_UpdateBodyLights :
    public SubscriberUpdateBodyLights
{
public:
    TapBrainstemCmd_UpdateBodyLights() :
        SubscriberUpdateBodyLights(ChuckTopics::driver::BRAINSTEM_CMD_UPDATE_BODY_LIGHTS)
    {}
};

} // namespace srs
