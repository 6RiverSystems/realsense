/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <srslib_framework/localization/map/Map.hpp>

namespace srs {

struct MapFactory
{
    static Map* fromRosCostMap2D(costmap_2d::Costmap2DROS* rosCostMap,
        unsigned int obstacleThreshold);
};

} // namespace srs
