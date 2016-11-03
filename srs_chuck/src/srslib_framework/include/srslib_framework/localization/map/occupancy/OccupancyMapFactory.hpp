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

    OccupancyMap* fromCostMap2D(costmap_2d::Costmap2D* costMap,
        double freeThreshold, double occupiedThreshold);
    OccupancyMap* fromGrid2d(Grid2d* grid, double resolution);
    OccupancyMap* fromMetadata(const OccupancyMetadata& metadata);
    OccupancyMap* fromMetadata(const OccupancyMetadata& metadata, const vector<int8_t>& occupancy);

private:
    void extractMonoChannel(SDL_Surface* image);
    void extractRGBChannel(SDL_Surface* image);
    void extractRGBAChannel(SDL_Surface* image);

    OccupancyMetadata metadata_;
    OccupancyMap* map_;
};

} // namespace srs
