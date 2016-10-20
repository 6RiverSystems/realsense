/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
#include <cmath>
using namespace std;

#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dNode.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dSingleGoal.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dPosition.hpp>

#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>
#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;

constexpr int GRID_SIZE = 5;

TEST(Test_AStar, SmallSearchWithWeights)
{
    Solution<Grid2dSolutionItem> correctSolution;
    correctSolution.push_back({Grid2dSolutionItem::ROTATE, Pose<>(3, 1, 0), Pose<>(3, 1, -M_PI_2), 4});
    correctSolution.push_back({Grid2dSolutionItem::MOVE, Pose<>(3, 1, -M_PI_2), Pose<>(3, 0, -M_PI_2), 5});
    correctSolution.push_back({Grid2dSolutionItem::ROTATE, Pose<>(3, 0, -M_PI_2), Pose<>(3, 0, -M_PI), 10});
    correctSolution.push_back({Grid2dSolutionItem::MOVE, Pose<>(3, 0, -M_PI), Pose<>(2, 0, -M_PI), 11});
    correctSolution.push_back({Grid2dSolutionItem::MOVE, Pose<>(2, 0, -M_PI), Pose<>(1, 0, -M_PI), 12});
    correctSolution.push_back({Grid2dSolutionItem::ROTATE, Pose<>(1, 0, -M_PI), Pose<>(1, 0, M_PI_2), 15});
    correctSolution.push_back({Grid2dSolutionItem::MOVE, Pose<>(1, 0, M_PI_2), Pose<>(1, 1, M_PI_2), 16});
    correctSolution.push_back({Grid2dSolutionItem::ROTATE, Pose<>(1, 1, M_PI_2), Pose<>(1, 1, 0), 18});

    Grid2d grid(GRID_SIZE, GRID_SIZE);
    grid.setWeights(Grid2d::Location(2, 1), 0, 0, 0, 100);
    grid.setWeights(Grid2d::Location(3, 1), 0, 0, 0, 100);

    LogicalMap* logical = LogicalMapFactory::fromGrid2d(&grid, 1);

    Grid2dPosition start(Grid2d::Location(3, 1), 0);
    Grid2dPosition goal(Grid2d::Location(1, 1), 0);

    Solution<Grid2dSolutionItem>* gridSolution = Grid2dSolutionFactory::fromSingleGoal(
        logical, start, goal);

    // The solution agrees with itself
    ASSERT_EQ(*gridSolution, *gridSolution) <<
        "The solution is not as expected";

    // The solution agrees with the correct solution
    ASSERT_EQ(correctSolution, *gridSolution) <<
        "The solution is not as expected";
}
