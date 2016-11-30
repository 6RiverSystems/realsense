/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>

using namespace srs;

TEST(Test_AStar_Grid2d, 6rshq_NoTrajectory)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("data/6rshq/6rshq.yaml");

    Pose<> start = Pose<>(3.000, 3.000, 0.0189141);
    Pose<> goal = Pose<>(3.000, 3.000, 0);

    Solution<Grid2dSolutionItem>* solution = Grid2dSolutionFactory::fromSingleGoal(
        mapStack->getLogicalMap(), start, goal);

    ASSERT_TRUE(solution->isValid()) <<
        "A solution was not found";

    ASSERT_TRUE(solution->empty()) <<
        "The solution found is not empty";

    ASSERT_EQ(1, solution->getExploredNodes()) <<
        "Unexpected explored nodes";
}
