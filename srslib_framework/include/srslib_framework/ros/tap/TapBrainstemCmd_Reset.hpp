/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberBoolean.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class TapBrainstemCmd_Reset :
    public SubscriberBoolean
{
public:
    TapBrainstemCmd_Reset() :
        SubscriberBoolean(ChuckTopics::driver::BRAINSTEM_CMD_RESET)
    {}
};

} // namespace srs
