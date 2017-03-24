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

    OccupancyMap* fromGrid2d(SimpleGrid2d* grid, Pose<> origin, double resolution);
    OccupancyMap* fromMetadata(const OccupancyMetadata& metadata);
    OccupancyMap* fromMetadata(const OccupancyMetadata& metadata, const vector<int8_t>& occupancy);
    OccupancyMap* fromMetadataRawCost(const OccupancyMetadata& metadata);

private:
    void extractMonoChannel(SDL_Surface* image);
    void extractMonoChannelRaw(SDL_Surface* image);
    void extractRGBChannel(SDL_Surface* image);
    void extractRGBAChannel(SDL_Surface* image);

    OccupancyMetadata metadata_;
    OccupancyMap* map_;
};

} // namespace srs
