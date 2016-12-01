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
#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;

constexpr int GRID_SIZE = 5;

MapStack* generateMapStack()
{
    Grid2d* grid = new Grid2d(GRID_SIZE, GRID_SIZE);
    test::Grid2dUtils::addObstacle(*grid, 2, 0, 2, 3);

    return test::Grid2dUtils::grid2d2MapStack(grid, 1, Pose<>::ZERO);
}

TEST(Test_AStar_MapStack, SmallSearchOnEmptyGrid)
{
    MapStack* mapStack = generateMapStack();

    AStar algorithm;

    Grid2d::Position startPosition(0, 0, 0);
    MapStackNode* start = MapStackNode::instanceOfStart(mapStack, startPosition);

    Grid2d::Position goalPosition(1, 1, 0);
    MapStackSingleGoal* goal = MapStackSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

    Plan plan;
    algorithm.getPlan(plan);
    cout << plan << endl;

    ASSERT_EQ(13, plan.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(5, plan.getOpenNodesCount()) <<
        "Unexpected number of open nodes";
}

TEST(Test_AStar_MapStack, Corner2CornerSearchOnEmptyGrid)
{
    MapStack* mapStack = generateMapStack();

    AStar algorithm;

    Grid2d::Position startPosition(0, 0, 0);
    MapStackNode* start = MapStackNode::instanceOfStart(mapStack, startPosition);

    Grid2d::Position goalPosition(GRID_SIZE - 1, GRID_SIZE - 1, 0);
    MapStackSingleGoal* goal = MapStackSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

    Plan plan;
    algorithm.getPlan(plan);
    cout << plan << endl;

    ASSERT_EQ(30, plan.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(17, plan.getOpenNodesCount()) <<
        "Unexpected number of open nodes";
}

TEST(Test_AStar_MapStack, SearchAroundObstacle)
{
    MapStack* mapStack = generateMapStack();

    AStar algorithm;

    Grid2d::Position startPosition(0, 0, 0);
    MapStackNode* start = MapStackNode::instanceOfStart(mapStack, startPosition);

    Grid2d::Position goalPosition(GRID_SIZE - 1, 0, 0);
    MapStackSingleGoal* goal = MapStackSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

    Plan plan;
    algorithm.getPlan(plan);
    cout << plan << endl;

    ASSERT_EQ(58, plan.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(18, plan.getOpenNodesCount()) <<
        "Unexpected number of open nodes";
}

TEST(Test_AStar_MapStack, MemoryLeaks)
{
    MapStack* mapStack = generateMapStack();

    Grid2d::Position startPosition(0, 0, 0);
    Grid2d::Position goalPosition(1, 1, 0);

    test::MemoryWatch memoryWatch;

    AStar* algorithm = new AStar();
    MapStackNode* start = MapStackNode::instanceOfStart(mapStack, startPosition);
    MapStackSingleGoal* goal = MapStackSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm->search(start, goal)) <<
        "A plan was not found";

    ASSERT_EQ(13, algorithm->getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(5, algorithm->getOpenNodesCount()) <<
        "Unexpected number of open nodes";

    algorithm->clear();

    start->release();
    goal->release();
    delete algorithm;

    ASSERT_TRUE(memoryWatch.isZero()) << "Memory leaks occurred";
}
