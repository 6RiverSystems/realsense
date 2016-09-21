/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/subscriber/SubscriberPoseStamped.hpp>

namespace srs {

class RosTapCmd_Goal :
    public SubscriberPoseStamped
{
public:
    RosTapCmd_Goal() :
        SubscriberPoseStamped("/request/goal")
    {}
};

} // namespace srs
