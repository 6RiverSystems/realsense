/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberOdometryRpm.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class TapBrainstemCmd_OdometryRpm :
    public SubscriberOdometryRpm
{
public:
    TapBrainstemCmd_OdometryRpm() :
        SubscriberOdometryRpm(ChuckTopics::driver::BRAINSTEM_ODOMETRY_RPM_CMD)
    {}
};

} // namespace srs
