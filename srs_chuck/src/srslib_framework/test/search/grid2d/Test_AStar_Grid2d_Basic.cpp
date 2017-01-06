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
#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dNode.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dSingleGoal.hpp>

#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>
#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;

constexpr int GRID_SIZE = 5;

TEST(Test_AStar_Grid2d_Basic, SmallSearchOnEmptyGrid)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);
    AStar algorithm;

    Position startPosition(0, 0, 0);
    Grid2dNode* start = Grid2dNode::instanceOfStart(&grid, startPosition);

    Position goalPosition(1, 1, 0);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

    ASSERT_EQ(13, algorithm.getOpenNodesCount()) <<
        "Unexpected number of open nodes";

    ASSERT_EQ(19, algorithm.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
}

TEST(Test_AStar_Grid2d_Basic, Corner2CornerSearchOnEmptyGrid)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);
    AStar algorithm;

    Position startPosition(0, 0, 0);
    Grid2dNode* start = Grid2dNode::instanceOfStart(&grid, startPosition);

    Position goalPosition(GRID_SIZE - 1, GRID_SIZE - 1, 0);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

    ASSERT_EQ(20, algorithm.getOpenNodesCount()) <<
        "Unexpected number of open nodes";

    ASSERT_EQ(79, algorithm.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
}

TEST(Test_AStar_Grid2d_Basic, SearchAroundObstacle)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);
    AStar algorithm;

    test::Grid2dUtils::addObstacle(grid, 2, 0, 2, 3);

    Position startPosition(0, 0, 0);
    Grid2dNode* start = Grid2dNode::instanceOfStart(&grid, startPosition);

    Position goalPosition(GRID_SIZE - 1, GRID_SIZE - 1, 0);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

    ASSERT_EQ(12, algorithm.getOpenNodesCount()) <<
        "Unexpected number of open nodes";

    ASSERT_EQ(35, algorithm.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
}

TEST(Test_AStar_Grid2d_Basic, Clear)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);
    AStar algorithm;

    Position startPosition(0, 0, 0);
    Grid2dNode* start = Grid2dNode::instanceOfStart(&grid, startPosition);

    Position goalPosition(1, 1, 0);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

    ASSERT_EQ(13, algorithm.getOpenNodesCount()) <<
        "Unexpected number of open nodes";

    ASSERT_EQ(19, algorithm.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";

    algorithm.clear();

    ASSERT_EQ(0, algorithm.getOpenNodesCount()) <<
        "Unexpected number of open nodes";

    ASSERT_EQ(0, algorithm.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

    ASSERT_EQ(13, algorithm.getOpenNodesCount()) <<
        "Unexpected number of open nodes";

    ASSERT_EQ(19, algorithm.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";

    start->release();
    goal->release();
}

TEST(Test_AStar_Grid2d_Basic, MemoryLeaks)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);

    Position startPosition(0, 0, 0);
    Position goalPosition(1, 1, 0);

    test::MemoryWatch memoryWatch;

    AStar* algorithm = new AStar();
    Grid2dNode* start = Grid2dNode::instanceOfStart(&grid, startPosition);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm->search(start, goal)) <<
        "A plan was not found";

    ASSERT_EQ(13, algorithm->getOpenNodesCount()) <<
        "Unexpected number of open nodes";

    ASSERT_EQ(19, algorithm->getClosedNodesCount()) <<
        "Unexpected number of closed nodes";

    algorithm->clear();

    start->release();
    goal->release();
    delete algorithm;

    ASSERT_TRUE(memoryWatch.isZero()) << "Memory leaks occurred";
}
