/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberRosOdometry.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class TapSensorOdometryPose :
    public SubscriberRosOdometry
{
public:
    TapSensorOdometryPose() :
        SubscriberRosOdometry(ChuckTopics::sensor::ODOMETRY_POSE)
    {}
};

} // namespace srs
