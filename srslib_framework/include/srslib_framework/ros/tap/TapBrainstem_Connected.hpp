/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberBoolean.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class TapBrainstem_Connected :
    public SubscriberBoolean
{
public:
    TapBrainstem_Connected() :
        SubscriberBoolean(ChuckTopics::driver::BRAINSTEM_STATE_CONNECTED)
    {}
};

} // namespace srs
