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
#include <srslib_framework/search/graph/grid2d/Grid2dPosition.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dSingleGoal.hpp>

using namespace srs;

TEST(Test_AStar, 6RHQ_NoTrajectory)
{
    Pose<> startPose = Pose<>(181335, 524097, 0.0189141);
    Pose<> goalPose = Pose<>(181335, 524097, 0);

    MapStack* mapStack = MapStackFactory::fromJsonFile("data/6rshq/6rshq.yaml");

    // Prepare the start position for the search
    Grid2d::Location internalStart;
    int startAngle;
    PoseAdapter::pose2Map(startPose, mapStack->getLogicalMap(), internalStart, startAngle);

    // Prepare the goal position for the search
    Grid2d::Location internalGoal;
    int goalAngle;
    PoseAdapter::pose2Map(goalPose, mapStack->getLogicalMap(), internalGoal, goalAngle);

    Grid2d* grid = mapStack->getLogicalMap()->getGrid();

    Grid2dPosition startPosition(internalStart, startAngle);
    Grid2dNode* start = Grid2dNode::instanceOfStart(grid, Grid2dPosition(internalStart, startAngle));

    Grid2dPosition goalPosition(internalGoal, goalAngle);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(Grid2dPosition(internalGoal, goalAngle));

    AStar algorithm;
    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A solution was not found";

    ASSERT_EQ(0, algorithm.getOpenNodeCount()) <<
        "Unexpected number of open nodes";

    ASSERT_EQ(1, algorithm.getClosedNodeCount()) <<
        "Unexpected number of closed nodes";
}
