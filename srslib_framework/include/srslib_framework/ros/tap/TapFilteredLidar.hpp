/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberLaserScan.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class TapFilteredLidar :
    public SubscriberLaserScan
{
public:
    TapFilteredLidar() :
        SubscriberLaserScan(ChuckTopics::sensor::FILTERED_LIDAR)
    {}
};

} // namespace srs
