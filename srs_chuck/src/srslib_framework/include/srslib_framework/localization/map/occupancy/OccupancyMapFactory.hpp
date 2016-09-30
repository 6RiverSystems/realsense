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

struct OccupancyMapFactory
{
    static OccupancyMap* fromMetadata(OccupancyMetadata metadata);
    static OccupancyMap* fromRosCostMap2D(costmap_2d::Costmap2DROS* rosCostMap,
        double freeThreshold, double occupiedThreshold);

    static void map2Occupancy(OccupancyMap* map, vector<int8_t>& occupancy);

    OccupancyMap* occupancy2Map(vector<int8_t>& occupancy);

private:
    static void extract1Channel(SDL_Surface* image, OccupancyMap* map);
    static void extract3Channel(SDL_Surface* image, OccupancyMap* map);
};

} // namespace srs
