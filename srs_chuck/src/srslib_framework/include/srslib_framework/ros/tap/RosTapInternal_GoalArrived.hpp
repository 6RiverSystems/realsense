/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/subscriber/SubscriberBoolean.hpp>

namespace srs {

class RosTapInternal_GoalArrived :
    public SubscriberBoolean
{
public:
    RosTapInternal_GoalArrived() :
        SubscriberBoolean("/internal/state/goal/arrived")
    {}
};

} // namespace srs
