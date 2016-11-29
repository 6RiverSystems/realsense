/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberString.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class TapExecutiveCmd_Cl :
    public SubscriberString
{
public:
    TapExecutiveCmd_Cl() :
        SubscriberString(ChuckTopics::node::EXECUTIVE_CMD_CL)
    {}
};

} // namespace srs
