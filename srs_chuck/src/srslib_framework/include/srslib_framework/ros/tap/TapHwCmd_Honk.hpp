/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberBoolean.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class TapHwCmd_Honk :
    public SubscriberBoolean
{
public:
    TapHwCmd_Honk() :
        SubscriberBoolean(ChuckTopics::internal::HWCMD_HONK)
    {}
};

} // namespace srs
