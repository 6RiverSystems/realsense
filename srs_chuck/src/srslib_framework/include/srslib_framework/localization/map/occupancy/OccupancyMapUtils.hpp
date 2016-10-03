/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <SDL/SDL_image.h>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <srslib_framework/localization/map/occupancy/OccupancyMetadata.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

namespace srs {

struct OccupancyMapUtils
{
    static void map2Occupancy(const OccupancyMap* map, vector<int8_t>& occupancy);

    static OccupancyMap* occupancy2Map(const OccupancyMetadata& metadata,
        const vector<int8_t>& occupancy);
};

} // namespace srs
