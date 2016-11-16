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
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackSingleGoal.hpp>

//#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>
//#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;

constexpr int GRID_SIZE = 5;

MapStack* generateMapStack()
{
    Grid2d* grid = new Grid2d(GRID_SIZE, GRID_SIZE);

    LogicalMapFactory logicalMapFactory;
    LogicalMap* logical = logicalMapFactory.fromGrid2d(grid, 1, Pose<>::ZERO);

    OccupancyMapFactory occupancyMapFactory;
    OccupancyMap* occupancy = occupancyMapFactory.fromGrid2d(grid, 1, Pose<>::ZERO);

    costmap_2d::Costmap2D* costMap2d = MapAdapter::map2CostMap2D(occupancy);

    return new MapStack(logical, occupancy, costMap2d);
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
        "A solution was not found";

    ASSERT_EQ(14, algorithm.getOpenNodeCount()) <<
        "Unexpected number of open nodes";

    ASSERT_EQ(16, algorithm.getClosedNodeCount()) <<
        "Unexpected number of closed nodes";
}

//TEST(Test_AStar, Corner2CornerSearchOnEmptyGrid)
//{
//    Grid2d grid(GRID_SIZE, GRID_SIZE);
//    AStar algorithm;
//
//    Grid2d::Position startPosition(0, 0, 0);
//    Grid2dNode* start = Grid2dNode::instanceOfStart(&grid, startPosition);
//
//    Grid2d::Position goalPosition(GRID_SIZE - 1, GRID_SIZE - 1, 0);
//    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);
//
//    ASSERT_TRUE(algorithm.search(start, goal)) <<
//        "A solution was not found";
//
//    ASSERT_EQ(44, algorithm.getOpenNodeCount()) <<
//        "Unexpected number of open nodes";
//
//    ASSERT_EQ(37, algorithm.getClosedNodeCount()) <<
//        "Unexpected number of closed nodes";
//}
//
//TEST(Test_AStar, SearchAroundObstacle)
//{
//    Grid2d grid(GRID_SIZE, GRID_SIZE);
//    AStar algorithm;
//
//    test::Grid2dUtils::addObstacle(grid, 2, 0, 2, 3);
//
//    Grid2d::Position startPosition(0, 0, 0);
//    Grid2dNode* start = Grid2dNode::instanceOfStart(&grid, startPosition);
//
//    Grid2d::Position goalPosition(GRID_SIZE - 1, GRID_SIZE - 1, 0);
//    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);
//
//    ASSERT_TRUE(algorithm.search(start, goal)) <<
//        "A solution was not found";
//
//    ASSERT_EQ(17, algorithm.getOpenNodeCount()) <<
//        "Unexpected number of open nodes";
//
//    ASSERT_EQ(30, algorithm.getClosedNodeCount()) <<
//        "Unexpected number of closed nodes";
//}
//
//TEST(Test_AStar, Clear)
//{
//    Grid2d grid(GRID_SIZE, GRID_SIZE);
//    AStar algorithm;
//
//    Grid2d::Position startPosition(0, 0, 0);
//    Grid2dNode* start = Grid2dNode::instanceOfStart(&grid, startPosition);
//
//    Grid2d::Position goalPosition(1, 1, 0);
//    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);
//
//    ASSERT_TRUE(algorithm.search(start, goal)) <<
//        "A solution was not found";
//
//    ASSERT_EQ(14, algorithm.getOpenNodeCount()) <<
//        "Unexpected number of open nodes";
//
//    ASSERT_EQ(16, algorithm.getClosedNodeCount()) <<
//        "Unexpected number of closed nodes";
//
//    algorithm.clear();
//
//    ASSERT_EQ(0, algorithm.getOpenNodeCount()) <<
//        "Unexpected number of open nodes";
//
//    ASSERT_EQ(0, algorithm.getClosedNodeCount()) <<
//        "Unexpected number of closed nodes";
//
//    ASSERT_TRUE(algorithm.search(start, goal)) <<
//        "A solution was not found";
//
//    ASSERT_EQ(12, algorithm.getOpenNodeCount()) <<
//        "Unexpected number of open nodes";
//
//    ASSERT_EQ(15, algorithm.getClosedNodeCount()) <<
//        "Unexpected number of closed nodes";
//
//    start->release();
//    goal->release();
//}
//
//TEST(Test_AStar, MemoryLeaks)
//{
//    Grid2d grid(GRID_SIZE, GRID_SIZE);
//
//    Grid2d::Position startPosition(0, 0, 0);
//    Grid2d::Position goalPosition(1, 1, 0);
//
//    test::MemoryWatch memoryWatch;
//
//    AStar* algorithm = new AStar();
//    Grid2dNode* start = Grid2dNode::instanceOfStart(&grid, startPosition);
//    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);
//
//    ASSERT_TRUE(algorithm->search(start, goal)) <<
//        "A solution was not found";
//
//    ASSERT_EQ(14, algorithm->getOpenNodeCount()) <<
//        "Unexpected number of open nodes";
//
//    ASSERT_EQ(16, algorithm->getClosedNodeCount()) <<
//        "Unexpected number of closed nodes";
//
//    algorithm->clear();
//
//    start->release();
//    goal->release();
//    delete algorithm;
//
//    ASSERT_TRUE(memoryWatch.isZero()) << "Memory leaks occurred";
//}
