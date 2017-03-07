/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberPoseWithCovarianceStamped.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class TapInitialPose :
    public SubscriberPoseWithCovarianceStamped
{
public:
    TapInitialPose() :
        SubscriberPoseWithCovarianceStamped(ChuckTopics::internal::ODOMETRY_INITIAL_POSE)
    {}
};

} // namespace srs
