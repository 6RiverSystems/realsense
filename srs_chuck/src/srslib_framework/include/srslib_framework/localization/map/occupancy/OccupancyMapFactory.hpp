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

class OccupancyMapFactory
{
public:
    OccupancyMapFactory() :
        map_(nullptr),
        metadata_()
    {}

    OccupancyMap* fromMetadata(OccupancyMetadata metadata);
    OccupancyMap* fromRosCostMap2D(costmap_2d::Costmap2DROS* rosCostMap,
        double freeThreshold, double occupiedThreshold);

private:
    void extract1Channel(SDL_Surface* image);
    void extract3Channel(SDL_Surface* image);

    OccupancyMetadata metadata_;
    OccupancyMap* map_;
};

} // namespace srs
