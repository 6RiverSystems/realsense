/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/datastructure/Position.hpp>
#include <srslib_framework/localization/map/MapAdapter.hpp>
#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>
#include <srslib_framework/planning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/trajectory/SimpleTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/trajectory/Trajectory.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>

#include <srsnode_navigation/global_planner/potentials/AStarPotentials.hpp>

#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>
using namespace srs;

constexpr int GRID_SIZE = 15;

TEST(Test_Navigation_AStarConventional, SmallSearch)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);
    test::Grid2dUtils::addRectanglePayload(grid, 0, 0, GRID_SIZE - 1, GRID_SIZE - 1, 0);

    MapStack* mapStack = test::Grid2dUtils::grid2d2MapStack(&grid, 0.05, Pose<>::ZERO);

    AStar algorithm;

    Position start(1, 1, 0);
    Position goal(5, 5, 0);

    Solution<Grid2dSolutionItem>* gridSolution = Grid2dSolutionFactory::fromSingleGoal(
        mapStack, start, goal);

    cout << *gridSolution << endl;

    ASSERT_TRUE(gridSolution->isValid()) <<
        "No valid solution was found";
    ASSERT_EQ(121, gridSolution->getExploredNodes()) <<
        "The solution is not as expected";
    ASSERT_EQ(10, gridSolution->getTotalCost()) <<
        "The total cost of the solution is not as expected";
}
