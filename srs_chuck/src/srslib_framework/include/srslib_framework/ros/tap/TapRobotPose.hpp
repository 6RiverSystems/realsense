/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberPose.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class TapRobotPose :
    public SubscriberPose
{
public:
    TapRobotPose() :
        SubscriberPose(ChuckTopics::internal::ROBOT_POSE)
    {}
};

} // namespace srs
