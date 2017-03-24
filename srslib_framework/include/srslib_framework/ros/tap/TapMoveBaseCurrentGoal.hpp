/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberPoseStamped.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class TapMoveBaseCurrentGoal :
    public SubscriberPoseStamped
{
public:
    TapMoveBaseCurrentGoal() :
        SubscriberPoseStamped(ChuckTopics::node::MOVE_BASE_CURRENT_GOAL)
    {}
};

} // namespace srs
