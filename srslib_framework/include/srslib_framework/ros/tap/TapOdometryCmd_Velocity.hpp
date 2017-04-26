/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberRosTwist.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class TapOdometryCmd_Velocity :
    public SubscriberRosTwist
{
public:
    TapOdometryCmd_Velocity() :
        SubscriberRosTwist(ChuckTopics::driver::ODOMETRY_CMD_VELOCITY)
    {}
};

} // namespace srs
