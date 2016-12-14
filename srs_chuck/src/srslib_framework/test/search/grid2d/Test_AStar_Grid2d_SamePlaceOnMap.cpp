/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dNode.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dSingleGoal.hpp>

using namespace srs;

TEST(Test_AStar_Grid2d_SamePlaceOnMap, SamePositionOnMap)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("data/6rshq/6rshq.yaml");

    AStar algorithm;

    Grid2d::Position startPosition(220, 260, 0);
    Grid2dNode* start = Grid2dNode::instanceOfStart(mapStack->getLogicalMap()->getGrid(),
        startPosition);

    Grid2d::Position goalPosition(220, 260, 0);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

    Plan plan;
    algorithm.getPlan(plan);
    cout << plan << endl;

    ASSERT_EQ(0, plan.getTotalCost()) <<
        "Unexpected cost of the path";
    ASSERT_EQ(1, plan.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(0, plan.getOpenNodesCount()) <<
        "Unexpected number of open nodes";
}

TEST(Test_AStar_Grid2d_SamePlaceOnMap, SameLocationOnMap)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("data/6rshq/6rshq.yaml");

    AStar algorithm;

    Grid2d::Position startPosition(220, 260, 0);
    Grid2dNode* start = Grid2dNode::instanceOfStart(mapStack->getLogicalMap()->getGrid(),
        startPosition);

    Grid2d::Position goalPosition(220, 260, 90);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

    Plan plan;
    algorithm.getPlan(plan);
    cout << plan << endl;

    ASSERT_EQ(2, plan.getTotalCost()) <<
        "Unexpected cost of the path";
    ASSERT_EQ(4, plan.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(5, plan.getOpenNodesCount()) <<
        "Unexpected number of open nodes";
}
