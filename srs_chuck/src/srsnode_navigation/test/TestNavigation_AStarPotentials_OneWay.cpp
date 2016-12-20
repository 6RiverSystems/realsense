/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/localization/map/MapAdapter.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_test/utils/Print.hpp>

#include <srsnode_navigation/global_planner/potentials/AStarPotentials.hpp>
using namespace srs;

constexpr int GRID_SIZE = 15;

TEST(Test_Navigation_AStarPotentials, OneWay)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("data/one-way/one-way.yaml");

    LogicalMap* logical = mapStack->getLogicalMap();
    costmap_2d::Costmap2D* cost2d = MapAdapter::map2CostMap2D(mapStack->getOccupancyMap());

    AStarPotentials astar(logical, cost2d);
    std::vector<std::pair<float, float>> path;
    float* potentials;

    AStarPotentials::SearchParameters searchParams;
    searchParams.allowUnknown = true;

    ASSERT_TRUE(astar.calculatePath(AStarPotentials::SearchParameters(),
        20, 20, 10, 20, path, potentials)) <<
        "No valid path was found";

    ASSERT_EQ(13, path.size()) << "The solution is not as expected";
}
