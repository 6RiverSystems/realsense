/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/subscriber/SubscriberPose.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class RosTapInternal_RobotPose :
    public SubscriberPose
{
public:
    RosTapInternal_RobotPose() :
        SubscriberPose(ChuckTopics::internal::ROBOT_POSE)
    {}
};

} // namespace srs
