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

#include <srslib_test/localization/map/MapStackUtils.hpp>

using namespace srs;

static constexpr unsigned int GRID_SIZE = 5;

TEST(Test_Solution_MapStack, BasicEmpty)
{
    Solution<Grid2dSolutionItem>* correctSolution = Solution<Grid2dSolutionItem>::instanceOfValidEmpty();
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(0, 0, 0), Pose<>(1, 0, 0), 1});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(1, 0, 0), Pose<>(1, 0, M_PI_2), 2});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(1, 0, M_PI_2), Pose<>(1, 1, M_PI_2), 3});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(1, 1, M_PI_2), Pose<>(1, 1, 0), 4});

    WeightedGrid2d logical(GRID_SIZE, GRID_SIZE);
    MapStack* mapStack = test::MapStackUtils::grid2d2MapStack(Pose<>::ZERO, 1, &logical);

    Pose<> start = Pose<>(0, 0, 0);
    Pose<> goal = Pose<>(1, 1, 0);

    Solution<Grid2dSolutionItem>* gridSolution = Grid2dSolutionFactory::fromSingleGoal(
        mapStack, start, goal);

    // The solution agrees with itself
    ASSERT_EQ(*gridSolution, *gridSolution) <<
        "The solution is not as expected";

    // The solution agrees with the correct solution
    ASSERT_EQ(*correctSolution, *gridSolution) <<
        "The solution is not as expected";
}
