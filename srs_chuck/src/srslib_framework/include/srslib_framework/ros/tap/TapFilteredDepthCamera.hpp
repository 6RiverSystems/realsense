/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberLaserScan.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class TapFilteredDepthCamera :
    public SubscriberLaserScan
{
public:
    TapFilteredDepthCamera() :
        SubscriberLaserScan(ChuckTopics::sensor::FILTERED_DEPTH_CAMERA)
    {}
};

} // namespace srs
