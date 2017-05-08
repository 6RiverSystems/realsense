/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>
using namespace std;

#include <costmap_2d/costmap_2d.h>

#include <srslib_framework/datastructure/Position.hpp>
#include <srslib_framework/localization/map/BaseMap.hpp>
#include <srslib_framework/localization/map/MapStackMetadata.hpp>
#include <srslib_framework/localization/map/logical/LogicalMap.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

namespace srs {

class MapStack
{
public:
    MapStack(MapStackMetadata metadata, LogicalMap* logical = nullptr,
        OccupancyMap* occupancy = nullptr,
        costmap_2d::Costmap2D* costMap2d = nullptr);
    ~MapStack();

    costmap_2d::Costmap2D* getCostMap2d() const;
    LogicalMap* getLogicalMap() const;
    MapStackMetadata getMetadata() const;
    bool getNeighbor(const Position& position, Position& result) const;
    OccupancyMap* getOccupancyMap() const;
    int getTotalCost(const Position& position,
        bool allowUnknown,
        float costMapRatio) const;
    int getWeight(const Position& position) const;

    void setCostMap2d(costmap_2d::Costmap2D* costMap2d);

private:
    MapStackMetadata metadata_;

    LogicalMap* logical_;

    OccupancyMap* occupancy_;

    costmap_2d::Costmap2D* costMap2d_;
};

} // namespace srs
