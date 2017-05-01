/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberUpdateToteLights.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class TapBrainstemCmd_UpdateToteLights :
    public SubscriberUpdateToteLights
{
public:
    TapBrainstemCmd_UpdateToteLights() :
        SubscriberUpdateToteLights(ChuckTopics::driver::BRAINSTEM_CMD_UPDATE_TOTE_LIGHTS)
    {}
};

} // namespace srs
