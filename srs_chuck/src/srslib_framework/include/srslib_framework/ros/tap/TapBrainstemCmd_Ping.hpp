/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberBoolean.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class TapBrainstemCmd_Ping :
    public SubscriberBoolean
{
public:
    TapBrainstemCmd_Ping() :
        SubscriberBoolean(ChuckTopics::driver::BRAINSTEM_CMD_PING)
    {}
};

} // namespace srs
