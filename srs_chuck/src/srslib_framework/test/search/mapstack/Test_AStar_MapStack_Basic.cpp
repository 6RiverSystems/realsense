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
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/Plan.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackSingleGoal.hpp>

#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>
#include <srslib_test/localization/map/MapStackUtils.hpp>
#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;

constexpr int GRID_SIZE = 5;

MapStack* generateMapStack()
{
    WeightedGrid2d* logical = new WeightedGrid2d(GRID_SIZE, GRID_SIZE);
    test::Grid2dUtils::addObstacle(logical, 2, 0, 2, 3);

    SimpleGrid2d* occupancy = new SimpleGrid2d(GRID_SIZE, GRID_SIZE);
    test::Grid2dUtils::addEmptySpace(occupancy);
    test::Grid2dUtils::addObstacle(occupancy, 2, 0, 2, 3);

    return test::MapStackUtils::grid2d2MapStack(Pose<>::ZERO, 1, logical, occupancy);
}

TEST(Test_AStar_MapStack_Basic, SmallSearchOnEmptyGrid)
{
    MapStack* mapStack = generateMapStack();

    AStar algorithm;

    Position startPosition(0, 0, 0);
    MapStackNode* start = MapStackNode::instanceOfStart(mapStack, startPosition);

    Position goalPosition(1, 1, 0);
    MapStackSingleGoal* goal = MapStackSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

    Plan plan;
    algorithm.getPlan(plan);
    cout << plan << endl;

    ASSERT_EQ(13, plan.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(4, plan.getOpenNodesCount()) <<
        "Unexpected number of open nodes";
}

TEST(Test_AStar_MapStack_Basic, Corner2CornerSearchOnEmptyGrid)
{
    MapStack* mapStack = generateMapStack();

    AStar algorithm;

    Position startPosition(0, 0, 0);
    MapStackNode* start = MapStackNode::instanceOfStart(mapStack, startPosition);

    Position goalPosition(GRID_SIZE - 1, GRID_SIZE - 1, 0);
    MapStackSingleGoal* goal = MapStackSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

    Plan plan;
    algorithm.getPlan(plan);
    cout << plan << endl;

    ASSERT_EQ(35, plan.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(12, plan.getOpenNodesCount()) <<
        "Unexpected number of open nodes";
}

TEST(Test_AStar_MapStack_Basic, SearchAroundObstacle)
{
    MapStack* mapStack = generateMapStack();

    AStar algorithm;

    Position startPosition(0, 0, 0);
    MapStackNode* start = MapStackNode::instanceOfStart(mapStack, startPosition);

    Position goalPosition(GRID_SIZE - 1, 0, 0);
    MapStackSingleGoal* goal = MapStackSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

    Plan plan;
    algorithm.getPlan(plan);
    cout << plan << endl;

    ASSERT_EQ(76, plan.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(8, plan.getOpenNodesCount()) <<
        "Unexpected number of open nodes";
}

TEST(Test_AStar_MapStack_Basic, MemoryLeaks)
{
    MapStack* mapStack = generateMapStack();

    Position startPosition(0, 0, 0);
    Position goalPosition(1, 1, 0);

    test::MemoryWatch memoryWatch;

    AStar* algorithm = new AStar();
    MapStackNode* start = MapStackNode::instanceOfStart(mapStack, startPosition);
    MapStackSingleGoal* goal = MapStackSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm->search(start, goal)) <<
        "A plan was not found";

    ASSERT_EQ(13, algorithm->getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(4, algorithm->getOpenNodesCount()) <<
        "Unexpected number of open nodes";

    algorithm->clear();

    start->release();
    goal->release();
    delete algorithm;

    ASSERT_TRUE(memoryWatch.isZero()) << "Memory leaks occurred";
}
