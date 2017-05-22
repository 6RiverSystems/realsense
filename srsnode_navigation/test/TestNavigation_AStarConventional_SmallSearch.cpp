/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/WeightedGrid2d.hpp>
#include <srslib_framework/datastructure/Position.hpp>
#include <srslib_framework/localization/map/MapAdapter.hpp>
#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>
#include <srslib_framework/planning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/trajectory/SimpleTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/trajectory/Trajectory.hpp>

#include <srslib_test/localization/map/MapStackUtils.hpp>

#include <srsnode_navigation/global_planner/potentials/AStarPotentials.hpp>

using namespace srs;

constexpr int GRID_SIZE = 15;

TEST(Test_Navigation_AStarConventional, SmallSearch)
{
    Solution<Grid2dSolutionItem>* correctSolution = Solution<Grid2dSolutionItem>::instanceOfValidEmpty();
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(1, 1, 0), Pose<>(2, 1, 0), 1});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(2, 1, 0), Pose<>(3, 1, 0), 2});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(3, 1, 0), Pose<>(4, 1, 0), 3});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(4, 1, 0), Pose<>(5, 1, 0), 4});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(5, 1, 0), Pose<>(5, 1, M_PI_2), 5});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(5, 1, M_PI_2), Pose<>(5, 2, M_PI_2), 6});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(5, 2, M_PI_2), Pose<>(5, 3, M_PI_2), 7});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(5, 3, M_PI_2), Pose<>(5, 4, M_PI_2), 8});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(5, 4, M_PI_2), Pose<>(5, 5, M_PI_2), 9});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(5, 5, M_PI_2), Pose<>(5, 5, 0), 10});

    WeightedGrid2d logical(GRID_SIZE, GRID_SIZE);
    MapStack* mapStack = test::MapStackUtils::grid2d2MapStack(Pose<>::ZERO, 1, &logical);

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

    // The solution agrees with itself
    ASSERT_EQ(*gridSolution, *gridSolution) <<
        "The solution is not as expected";

    // The solution agrees with the correct solution
    ASSERT_EQ(*correctSolution, *gridSolution) <<
        "The solution is not as expected";
}
