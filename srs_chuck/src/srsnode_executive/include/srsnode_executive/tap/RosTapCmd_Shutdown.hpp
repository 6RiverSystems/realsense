/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberBoolean.hpp>

namespace srs {

class RosTapCmd_Shutdown :
    public SubscriberBoolean
{
public:
    RosTapCmd_Shutdown() :
        SubscriberBoolean("/request/shutdown")
    {}
};

} // namespace srs
