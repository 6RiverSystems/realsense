/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberPoseStamped.hpp>

namespace srs {

class RosTapCmd_InitialPose :
    public SubscriberPoseStamped
{
public:
    RosTapCmd_InitialPose() :
        SubscriberPoseStamped("/request/initial_pose")
    {}
};

} // namespace srs
