/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
using namespace std;

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapAdapter.hpp>
#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/Plan.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackSingleGoal.hpp>

#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>
using namespace srs;

constexpr int GRID_SIZE = 5;

TEST(Test_AStar_MapStack, NoSolution)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);
    test::Grid2dUtils::addObstacle(grid, 0, 0, GRID_SIZE - 1, GRID_SIZE - 1);

    MapStack* mapStack = test::Grid2dUtils::grid2d2MapStack(&grid, 1, Pose<>::ZERO);

    AStar algorithm;

    Grid2d::Position startPosition(0, 0, 0);
    MapStackNode* start = MapStackNode::instanceOfStart(mapStack, startPosition);

    Grid2d::Position goalPosition(1, 1, 0);
    MapStackSingleGoal* goal = MapStackSingleGoal::instanceOf(goalPosition);

    ASSERT_FALSE(algorithm.search(start, goal)) <<
        "A solution was found";

    ASSERT_EQ(1, algorithm.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(0, algorithm.getOpenNodesCount()) <<
        "Unexpected number of open nodes";
}

TEST(Test_AStar_MapStack, ForbiddenGoal)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);
    test::Grid2dUtils::addObstacle(grid, 1, 0, GRID_SIZE - 1, GRID_SIZE - 1);

    MapStack* mapStack = test::Grid2dUtils::grid2d2MapStack(&grid, 1, Pose<>::ZERO);

    AStar algorithm;

    Grid2d::Position startPosition(0, 0, 0);
    MapStackNode* start = MapStackNode::instanceOfStart(mapStack, startPosition);

    Grid2d::Position goalPosition(GRID_SIZE - 1, 0, 0);
    MapStackSingleGoal* goal = MapStackSingleGoal::instanceOf(goalPosition);

    ASSERT_FALSE(algorithm.search(start, goal)) <<
        "A solution was found";

    ASSERT_EQ(20, algorithm.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(0, algorithm.getOpenNodesCount()) <<
        "Unexpected number of open nodes";
}

TEST(Test_AStar_MapStack, UnreachableGoal)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);
    test::Grid2dUtils::addObstacle(grid, 2, 0, 2, GRID_SIZE - 1);

    MapStack* mapStack = test::Grid2dUtils::grid2d2MapStack(&grid, 1, Pose<>::ZERO);

    AStar algorithm;

    Grid2d::Position startPosition(0, 0, 0);
    MapStackNode* start = MapStackNode::instanceOfStart(mapStack, startPosition);

    Grid2d::Position goalPosition(GRID_SIZE - 1, 0, 0);
    MapStackSingleGoal* goal = MapStackSingleGoal::instanceOf(goalPosition);

    ASSERT_FALSE(algorithm.search(start, goal)) <<
        "A solution was found";

    ASSERT_EQ(40, algorithm.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(0, algorithm.getOpenNodesCount()) <<
        "Unexpected number of open nodes";
}
