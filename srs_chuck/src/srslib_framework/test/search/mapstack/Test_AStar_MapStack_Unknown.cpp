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

MapStack* generateMapStackWithUnknown()
{
    WeightedGrid2d* logical = new WeightedGrid2d(GRID_SIZE, GRID_SIZE);
    test::Grid2dUtils::addObstacle(logical, 2, 0, 2, 3);

    SimpleGrid2d* occupancy = new SimpleGrid2d(GRID_SIZE, GRID_SIZE);

    return test::MapStackUtils::grid2d2MapStack(Pose<>::ZERO, 1, logical, occupancy);
}

TEST(Test_AStar_MapStack_Unknown, AllowUnknown)
{
    MapStack* mapStack = generateMapStackWithUnknown();

    AStar algorithm;

    MapStackNode::SearchParameters searchParams;
    searchParams.allowUnknown = true;

    Position startPosition(0, 0, 0);
    MapStackNode* start = MapStackNode::instanceOfStart(mapStack, startPosition, searchParams);

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

TEST(Test_AStar_MapStack_Unknown, NotAllowUnknown)
{
    MapStack* mapStack = generateMapStackWithUnknown();

    AStar algorithm;

    MapStackNode::SearchParameters searchParams;
    searchParams.allowUnknown = false;

    Position startPosition(0, 0, 0);
    MapStackNode* start = MapStackNode::instanceOfStart(mapStack, startPosition, searchParams);

    Position goalPosition(1, 1, 0);
    MapStackSingleGoal* goal = MapStackSingleGoal::instanceOf(goalPosition);

    ASSERT_FALSE(algorithm.search(start, goal)) <<
        "A plan was found";
}