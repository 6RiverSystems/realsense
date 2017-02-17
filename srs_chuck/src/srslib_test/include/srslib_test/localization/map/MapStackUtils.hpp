/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <iostream>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/BaseGrid2d.hpp>
#include <srslib_framework/datastructure/graph/grid2d/SimpleGrid2d.hpp>
#include <srslib_framework/datastructure/graph/grid2d/WeightedGrid2d.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackMetadata.hpp>
#include <srslib_framework/localization/map/MapAdapter.hpp>
#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>

namespace srs {
namespace test {

struct MapStackUtils
{
    static MapStack* grid2d2MapStack(Pose<> origin, double resolution,
        WeightedGrid2d* logicalGrid = nullptr, SimpleGrid2d* occupancyGrid = nullptr)
    {
        LogicalMap* logicalMap = nullptr;
        OccupancyMap* occupancyMap = nullptr;
        costmap_2d::Costmap2D* costMap = nullptr;

        if (logicalGrid)
        {
            LogicalMapFactory logicalMapFactory;
            logicalMap = logicalMapFactory.fromGrid2d(logicalGrid, origin, resolution);

        }

        if (occupancyGrid)
        {
            OccupancyMapFactory occupancyMapFactory;
            occupancyMap = occupancyMapFactory.fromGrid2d(occupancyGrid, origin, resolution);

            costMap = MapAdapter::map2CostMap2D(occupancyMap);
        }
        else
        {
            if (logicalMap)
            {
                costMap = MapAdapter::map2CostMap2D(logicalMap);
            }
        }

        MapStackMetadata metadata(0, "MapStackUtils", "Test map", "1.0");

        return new MapStack(metadata, logicalMap, occupancyMap, costMap);
    }
};

} // namespace test
} // namespace srs
