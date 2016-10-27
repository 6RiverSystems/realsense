/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dNode.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dSingleGoal.hpp>

#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>
#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;

constexpr int GRID_SIZE = 5;

TEST(Test_AStar, NoSolution)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);
    test::Grid2dUtils::addStaticObstacle(grid, 0, 0, GRID_SIZE - 1, GRID_SIZE - 1);

    AStar algorithm;

    Grid2d::Position startPosition(0, 0, 0);
    Grid2dNode* start = Grid2dNode::instanceOfStart(&grid, startPosition);

    Grid2d::Position goalPosition(1, 1, 0);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    ASSERT_FALSE(algorithm.search(start, goal)) <<
        "A solution was found";

    ASSERT_EQ(0, algorithm.getOpenNodeCount()) <<
        "Unexpected number of open nodes";

    ASSERT_EQ(1, algorithm.getClosedNodeCount()) <<
        "Unexpected number of closed nodes";
}

TEST(Test_AStar, ForbiddenGoal)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);
    AStar algorithm;

    test::Grid2dUtils::addStaticObstacle(grid, 1, 0, GRID_SIZE - 1, GRID_SIZE - 1);

    Grid2d::Position startPosition(0, 0, 0);
    Grid2dNode* start = Grid2dNode::instanceOfStart(&grid, startPosition);

    Grid2d::Position goalPosition(GRID_SIZE - 1, 0, 0);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    ASSERT_FALSE(algorithm.search(start, goal)) <<
        "A solution was found";

    ASSERT_EQ(0, algorithm.getOpenNodeCount()) <<
        "Unexpected number of open nodes";

    ASSERT_EQ(20, algorithm.getClosedNodeCount()) <<
        "Unexpected number of closed nodes";
}

TEST(Test_AStar, UnreachableGoal)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);
    AStar algorithm;

    test::Grid2dUtils::addStaticObstacle(grid, 2, 0, 2, GRID_SIZE - 1);

    Grid2d::Position startPosition(0, 0, 0);
    Grid2dNode* start = Grid2dNode::instanceOfStart(&grid, startPosition);

    Grid2d::Position goalPosition(GRID_SIZE - 1, 0, 0);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    ASSERT_FALSE(algorithm.search(start, goal)) <<
        "A solution was found";

    ASSERT_EQ(0, algorithm.getOpenNodeCount()) <<
        "Unexpected number of open nodes";

    ASSERT_EQ(40, algorithm.getClosedNodeCount()) <<
        "Unexpected number of closed nodes";
}
