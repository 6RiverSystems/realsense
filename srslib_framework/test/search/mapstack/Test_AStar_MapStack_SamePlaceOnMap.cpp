/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/datastructure/Position.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/Plan.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackSingleGoal.hpp>
using namespace srs;

TEST(Test_AStar_MapStack_SamePlaceOnMap, SamePositionOnMap)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("data/6rshq/6rshq.yaml");

    AStar algorithm;

    Position startPosition(220, 260, 0);
    MapStackNode* start = MapStackNode::instanceOfStart(mapStack, startPosition);

    Position goalPosition(220, 260, 0);
    MapStackSingleGoal* goal = MapStackSingleGoal::instanceOf(goalPosition);

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

TEST(Test_AStar_MapStack_SamePlaceOnMap, SameLocationOnMap)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile(
        "search/data/6rshq/6rshq.yaml");

    AStar algorithm;

    Position startPosition(220, 260, 0);
    MapStackNode* start = MapStackNode::instanceOfStart(mapStack, startPosition);

    Position goalPosition(220, 260, 90);
    MapStackSingleGoal* goal = MapStackSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

    Plan plan;
    algorithm.getPlan(plan);
    cout << plan << endl;

    ASSERT_EQ(1, plan.getTotalCost()) <<
        "Unexpected cost of the path";
    ASSERT_EQ(3, plan.getClosedNodesCount()) <<
        "Unexpected number of closed nodes";
    ASSERT_EQ(3, plan.getOpenNodesCount()) <<
        "Unexpected number of open nodes";
}
