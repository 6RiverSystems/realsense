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
#include <srslib_framework/search/graph/grid2d/Grid2dPosition.hpp>

#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>
using namespace srs;

constexpr int GRID_SIZE = 5;

TEST(Test_AStar, SmallSearchOnEmptyGrid)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);
    AStar<Grid2d::LocationHash, Grid2d::LocationEqual> algorithm;

    Grid2dPosition goalPosition(Grid2d::Location(1, 1), 0);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    Grid2dPosition startPosition(Grid2d::Location(0, 0), 0);
    Grid2dNode* start = Grid2dNode::instanceOfStart(&grid, startPosition, goal);

    ASSERT_TRUE(algorithm.search(start)) <<
        "A solution for an empty grid was not found";
}

TEST(Test_AStar, Corner2CornerSearchOnEmptyGrid)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);
    AStar<Grid2d::LocationHash, Grid2d::LocationEqual> algorithm;

    Grid2dPosition goalPosition(Grid2d::Location(GRID_SIZE - 1, GRID_SIZE - 1), 0);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    Grid2dPosition startPosition(Grid2d::Location(0, 0), 0);
    Grid2dNode* start = Grid2dNode::instanceOfStart(&grid, startPosition, goal);

    ASSERT_TRUE(algorithm.search(start)) <<
        "A solution for an empty grid was not found";
}

TEST(Test_AStar, SearchAroundObstacle)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);
    AStar<Grid2d::LocationHash, Grid2d::LocationEqual> algorithm;

    test::Grid2dUtils::addStaticObstacle(grid, 2, 0, 2, 3);

    cout << grid << endl;

    Grid2dPosition goalPosition(Grid2d::Location(GRID_SIZE - 1, GRID_SIZE - 1), 0);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    Grid2dPosition startPosition(Grid2d::Location(0, 0), 0);
    Grid2dNode* start = Grid2dNode::instanceOfStart(&grid, startPosition, goal);

    ASSERT_TRUE(algorithm.search(start)) <<
        "A solution for an empty grid was not found";
}
