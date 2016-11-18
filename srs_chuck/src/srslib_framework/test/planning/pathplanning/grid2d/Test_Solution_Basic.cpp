/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
#include <cmath>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>

using namespace srs;

static constexpr unsigned int GRID_SIZE = 5;

TEST(Test_Solution, BasicEmptyGrid)
{
    Solution<Grid2dSolutionItem> correctSolution;
    correctSolution.push_back({Grid2dSolutionItem::MOVE, Pose<>(0, 0, 0), Pose<>(1, 0, 0), 1});
    correctSolution.push_back({Grid2dSolutionItem::ROTATE, Pose<>(1, 0, 0), Pose<>(1, 0, M_PI_2), 3});
    correctSolution.push_back({Grid2dSolutionItem::MOVE, Pose<>(1, 0, M_PI_2), Pose<>(1, 1, M_PI_2), 4});
    correctSolution.push_back({Grid2dSolutionItem::ROTATE, Pose<>(1, 1, M_PI_2), Pose<>(1, 1, 0), 6});

    Grid2d grid(GRID_SIZE, GRID_SIZE);

    Pose<> start = Pose<>(0, 0, 0);
    Pose<> goal = Pose<>(1, 1, 0);

    LogicalMapFactory logicalMapFactory;
    LogicalMap* logical = logicalMapFactory.fromGrid2d(&grid, 1, Pose<>::ZERO);

    Solution<Grid2dSolutionItem>* gridSolution = Grid2dSolutionFactory::fromSingleGoal(
        logical, start, goal);

    // The solution agrees with itself
    ASSERT_EQ(*gridSolution, *gridSolution) <<
        "The solution is not as expected";

    // The solution agrees with the correct solution
    ASSERT_EQ(correctSolution, *gridSolution) <<
        "The solution is not as expected";
}
