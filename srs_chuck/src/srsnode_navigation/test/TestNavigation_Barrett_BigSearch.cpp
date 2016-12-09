/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <srslib_framework/localization/map/MapAdapter.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_framework/localization/map/logical/LogicalMap.hpp>
#include <srslib_framework/platform/timing/StopWatch.hpp>

#include <srsnode_navigation/global_planner/AStarPotentials.hpp>
using namespace srs;

TEST(Test_Navigation, Barrett_BigSearch)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("data/barrett/barrett.yaml");

    // Looking for a path between
    // Pose {@: 1.4686e+09, x: 14.7855, y: 4.65954, t: 1.53943} (148, 47, 90)
    // and
    // Pose {@: 1.4686e+09, x: 33.215, y: 51.703, t: 1.5708} (332, 522, 90)
    Grid2d::Position startPosition(148, 47, 90);
    Grid2d::Position goalPosition(332, 522, 90);

    LogicalMap* logical = mapStack->getLogicalMap();

    costmap_2d::Costmap2D* cost2d = MapAdapter::map2CostMap2D(mapStack->getOccupancyMap());

    AStarPotentials astar(logical, cost2d);
    std::vector<std::pair<float, float>> path;
    float* potentials;

    StopWatch timer;

    if (!astar.calculatePath(13.804, 4.478, 32.679, 87.743, path, potentials))
    {
        cout << "Path not found" << endl;
    }
    cout << "Elapsed time: " << timer.elapsed() << "s" << endl;
}
