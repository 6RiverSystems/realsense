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
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dNode.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dSingleGoal.hpp>

#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;

TEST(Test_AStar_Grid2d_6rshq, SmallSearch)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("data/6rshq/6rshq.yaml");

    Position startPosition(220, 260, 0);
    Position goalPosition(222, 260, 0);

    test::MemoryWatch memoryWatch;

    AStar* algorithm = new AStar();
    Grid2dNode* start = Grid2dNode::instanceOfStart(mapStack->getLogicalMap()->getGrid(),
        startPosition);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm->search(start, goal)) <<
        "A plan was not found";

    ASSERT_EQ(4, algorithm->getOpenNodesCount()) <<
        "Unexpected number of open nodes";

    ASSERT_EQ(3, algorithm->getClosedNodesCount()) <<
        "Unexpected number of closed nodes";

    algorithm->clear();

    start->release();
    goal->release();
    delete algorithm;

    ASSERT_TRUE(memoryWatch.isZero()) << "Memory leaks occurred";
}

TEST(Test_AStar_Grid2d_6rshq, NoTrajectory)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("data/6rshq/6rshq.yaml");

    Pose<> start = Pose<>(22.000, 26.000, 0.0189141);
    Pose<> goal = Pose<>(22.000, 26.000, 0);

    Solution<Grid2dSolutionItem>* solution = Grid2dSolutionFactory::fromSingleGoal(
        mapStack->getLogicalMap(), start, goal);

    ASSERT_TRUE(solution->isValid()) <<
        "A solution was not found";

    ASSERT_TRUE(solution->empty()) <<
        "The solution found is not empty";

    ASSERT_EQ(1, solution->getExploredNodes()) <<
        "Unexpected explored nodes";
}
