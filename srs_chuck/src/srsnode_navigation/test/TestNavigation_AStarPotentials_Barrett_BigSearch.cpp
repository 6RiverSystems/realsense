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
#include <srslib_timing/StopWatch.hpp>

#include <srsnode_navigation/global_planner/potentials/AStarPotentials.hpp>
using namespace srs;

static const int TRIALS = 10;

TEST(Test_Navigation_AStarPotentials, Barrett_BigSearch)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("data/barrett/barrett.yaml");

    // Looking for a path between
    // Pose {@: 1.4686e+09, x: 14.0, y: 10.0, t: 1.53943} (140, 100, 90)
    // and
    // Pose {@: 1.4686e+09, x: 73.0, y: 178.0, t: 1.5708} (730, 1780, 90)

    LogicalMap* logical = mapStack->getLogicalMap();

    costmap_2d::Costmap2D* cost2d = MapAdapter::map2CostMap2D(mapStack->getOccupancyMap());

    AStarPotentials astar(logical, cost2d);
    std::vector<std::pair<float, float>> path;
    float* potentials;

    ASSERT_TRUE(astar.calculatePath(AStarPotentials::SearchParameters(),
        14.000, 10.000, 73.000, 178.000, path, potentials)) <<
        "A plan was not found";

    StopWatch timer;

    int trials = TRIALS;
    while (--trials > 0)
    {
        astar.calculatePath(AStarPotentials::SearchParameters(),
            14.000, 10.000, 73.000, 178.000, path, potentials);
    }

    float elapsed = timer.elapsedMilliseconds();

    cout << "Elapsed time: " << elapsed << "ms" << endl;
    cout << "Average time: " << elapsed / TRIALS << "ms" << endl;
}
