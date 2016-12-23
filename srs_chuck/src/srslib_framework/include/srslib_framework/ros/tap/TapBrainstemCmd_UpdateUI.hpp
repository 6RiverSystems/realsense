/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberUpdateUI.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class TapBrainstemCmd_UpdateUI :
    public SubscriberUpdateUI
{
public:
    TapBrainstemCmd_UpdateUI() :
        SubscriberUpdateUI(ChuckTopics::driver::BRAINSTEM_CMD_UPDATE_LIGHTS)
    {}
};

} // namespace srs
