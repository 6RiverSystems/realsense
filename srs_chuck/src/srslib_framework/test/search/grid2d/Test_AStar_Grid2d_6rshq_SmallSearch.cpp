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
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dNode.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dSingleGoal.hpp>

#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;

TEST(Test_AStar_Grid2d, 6rshq_SmallSearch)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("data/6rshq/6rshq.yaml");

    Grid2d::Position startPosition(86, 48, 0);
    Grid2d::Position goalPosition(87, 48, 0);

    test::MemoryWatch memoryWatch;

    AStar* algorithm = new AStar();
    Grid2dNode* start = Grid2dNode::instanceOfStart(mapStack->getLogicalMap()->getGrid(),
        startPosition);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm->search(start, goal)) <<
        "A plan was not found";

    ASSERT_EQ(2, algorithm->getOpenNodesCount()) <<
        "Unexpected number of open nodes";

    ASSERT_EQ(2, algorithm->getClosedNodesCount()) <<
        "Unexpected number of closed nodes";

    algorithm->clear();

    start->release();
    goal->release();
    delete algorithm;

    ASSERT_TRUE(memoryWatch.isZero()) << "Memory leaks occurred";
}
