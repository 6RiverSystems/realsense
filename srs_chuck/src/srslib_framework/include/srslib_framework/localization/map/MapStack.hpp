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

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/localization/map/BaseMap.hpp>
#include <srslib_framework/localization/map/logical/LogicalMap.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

namespace srs {

class MapStack
{
public:
    MapStack(LogicalMap* logical = nullptr,
        OccupancyMap* occupancy = nullptr,
        costmap_2d::Costmap2D* costMap2d = nullptr);
    ~MapStack();

    costmap_2d::Costmap2D* getCostMap2d() const;
    LogicalMap* getLogicalMap() const;
    bool getNeighbor(const Grid2d::Position& position, Grid2d::Position& result) const;
    OccupancyMap* getOccupancyMap() const;
    int getTotalCost(const Grid2d::Position& position,
        bool allowUnknown,
        float costMapRatio) const;
    int getWeight(const Grid2d::Position& position) const;

    void setCostMap2d(costmap_2d::Costmap2D* costMap2d);

private:
    LogicalMap* logical_;
    OccupancyMap* occupancy_;
    costmap_2d::Costmap2D* costMap2d_;
};

} // namespace srs