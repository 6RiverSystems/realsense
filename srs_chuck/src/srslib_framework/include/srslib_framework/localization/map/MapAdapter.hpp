/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <srslib_framework/localization/map/occupancy/OccupancyMetadata.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

namespace srs {

struct MapAdapter
{
    static void costMap2D2Vector(const costmap_2d::Costmap2D* map, vector<int8_t>& occupancy);

    static costmap_2d::Costmap2D* map2CostMap2D(OccupancyMap* map);

    /**
     * @brief Convert a Occupancy Map type into a vector of integers.
     *
     * @param map Occupancy Map to convert
     * @param occupancy Reference to the vector of integers
     */
    static void occupancyMap2Vector(const OccupancyMap* map, vector<int8_t>& occupancy);

    static OccupancyMap* vector2Map(const OccupancyMetadata& metadata,
        const vector<int8_t>& occupancy);

    static costmap_2d::Costmap2D* weights2CostMap2D(BaseMap* map, int orientation);
};

} // namespace srs
