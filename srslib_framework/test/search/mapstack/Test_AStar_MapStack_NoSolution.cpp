/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
using namespace std;

#include <srslib_framework/datastructure/Position.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapAdapter.hpp>
#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackAStar.hpp>
#include <srslib_framework/search/Plan.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackSingleGoal.hpp>

#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>
#include <srslib_test/localization/map/MapStackUtils.hpp>

using namespace srs;

constexpr int GRID_SIZE = 5;

TEST(Test_AStar_MapStack_NoSolution, NoSolution)
{
    WeightedGrid2d logical(GRID_SIZE, GRID_SIZE);
    test::Grid2dUtils::addObstacle(logical, 0, 0, GRID_SIZE - 1, GRID_SIZE - 1);

    MapStack* mapStack = test::MapStackUtils::grid2d2MapStack(Pose<>::ZERO, 1, &logical);

    MapStackAStar algorithm(mapStack);

    Position start(0, 0, 0);
    Position goal(1, 1, 0);

    ASSERT_FALSE(algorithm.search(start, goal)) <<
        "A solution was found";

    ASSERT_EQ(1, algorithm.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(0, algorithm.getOpenNodesCount()) <<
        "Unexpected number of open nodes";
}

TEST(Test_AStar_MapStack_NoSolution, ForbiddenGoal)
{
    WeightedGrid2d logical(GRID_SIZE, GRID_SIZE);
    test::Grid2dUtils::addObstacle(logical, 1, 0, GRID_SIZE - 1, GRID_SIZE - 1);

    MapStack* mapStack = test::MapStackUtils::grid2d2MapStack(Pose<>::ZERO, 1, &logical);

    MapStackAStar algorithm(mapStack);

    Position start(0, 0, 0);
    Position goal(GRID_SIZE - 1, 0, 0);

    ASSERT_FALSE(algorithm.search(start, goal)) <<
        "A solution was found";

    ASSERT_EQ(20, algorithm.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(0, algorithm.getOpenNodesCount()) <<
        "Unexpected number of open nodes";
}

TEST(Test_AStar_MapStack_NoSolution, UnreachableGoal)
{
    WeightedGrid2d logical(GRID_SIZE, GRID_SIZE);
    test::Grid2dUtils::addObstacle(logical, 2, 0, 2, GRID_SIZE - 1);

    MapStack* mapStack = test::MapStackUtils::grid2d2MapStack(Pose<>::ZERO, 1, &logical);

    MapStackAStar algorithm(mapStack);

    Position start(0, 0, 0);
    Position goal(GRID_SIZE - 1, 0, 0);

    ASSERT_FALSE(algorithm.search(start, goal)) <<
        "A solution was found";

    ASSERT_EQ(40, algorithm.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(0, algorithm.getOpenNodesCount()) <<
        "Unexpected number of open nodes";
}
