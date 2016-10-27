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
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/PoseAdapter.hpp>
#include <srslib_framework/robotics/robot_profile/ChuckProfile.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dNode.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dSingleGoal.hpp>

using namespace srs;

TEST(Test_AStar, 6RHQ_NoTrajectory)
{
    Pose<> startPose = Pose<>(181335, 524097, 0.0189141);
    Pose<> goalPose = Pose<>(181335, 524097, 0);

    MapStack* mapStack = MapStackFactory::fromJsonFile("data/6rshq/6rshq.yaml");

    // Prepare the start position for the search
    Grid2d::Position startPosition = PoseAdapter::pose2Map(startPose, mapStack->getLogicalMap());

    // Prepare the goal position for the search
    Grid2d::Position goalPosition = PoseAdapter::pose2Map(goalPose, mapStack->getLogicalMap());

    Grid2d* grid = mapStack->getLogicalMap()->getGrid();

    Grid2dNode* start = Grid2dNode::instanceOfStart(grid, startPosition);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    AStar algorithm;
    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A solution was not found";

    ASSERT_EQ(0, algorithm.getOpenNodeCount()) <<
        "Unexpected number of open nodes";

    ASSERT_EQ(1, algorithm.getClosedNodeCount()) <<
        "Unexpected number of closed nodes";
}
