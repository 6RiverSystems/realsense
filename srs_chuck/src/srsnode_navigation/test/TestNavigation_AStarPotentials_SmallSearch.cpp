/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/localization/map/MapAdapter.hpp>
#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srsnode_navigation/global_planner/potentials/AStarPotentials.hpp>

#include <srslib_test/localization/map/MapStackUtils.hpp>

using namespace srs;

constexpr int GRID_SIZE = 15;

TEST(Test_Navigation_AStarPotentials, SmallSearch)
{
    WeightedGrid2d logical(GRID_SIZE, GRID_SIZE);
    MapStack* mapStack = test::MapStackUtils::grid2d2MapStack(Pose<>::ZERO, 1, &logical);

    AStarPotentials astar(mapStack->getLogicalMap(), mapStack->getCostMap2d(), AStarPotentials::QueueMapType());

    std::vector<std::pair<float, float>> path;
    float* potentials;

    ASSERT_TRUE(astar.calculatePath(AStarPotentials::SearchParameters(),
        1, 1, 5, 5, path, potentials)) << "No valid path was found";

    ASSERT_EQ(5, path.size()) << "The solution is not as expected";
}
