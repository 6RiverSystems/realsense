/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>

#include <srslib_test/localization/map/MapStackUtils.hpp>

using namespace srs;

static constexpr double DEG0 = 0.0;
static constexpr double DEG90 = AngleMath::deg2Rad<double>(90);
static constexpr double DEG180 = AngleMath::deg2Rad<double>(180);

static constexpr unsigned int GRID_SIZE = 5;

TEST(Test_Solution_MapStack, ConsecutiveEmpty)
{
    Solution<Grid2dSolutionItem>* correctSolution = Solution<Grid2dSolutionItem>::instanceOfValidEmpty();
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(0, 0, 0), Pose<>(1, 0, 0), 1});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(1, 0, 0), Pose<>(1, 0, M_PI_2), 2});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(1, 0, M_PI_2), Pose<>(1, 1, M_PI_2), 3});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(1, 1, M_PI_2), Pose<>(1, 1, 0), 4});

    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(1, 1, 0), Pose<>(1, 1, M_PI_2), 1});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(1, 1, M_PI_2), Pose<>(1, 2, M_PI_2), 2});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(1, 2, M_PI_2), Pose<>(1, 3, M_PI_2), 3});

    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(1, 3, M_PI_2), Pose<>(1, 3, 0), 1});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(1, 3, 0), Pose<>(2, 3, 0), 2});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(2, 3, 0), Pose<>(3, 3, 0), 3});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(3, 3, 0), Pose<>(3, 3, -M_PI_2), 4});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(3, 3, -M_PI_2), Pose<>(3, 3, -M_PI), 5});

    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(3, 3, -M_PI), Pose<>(3, 3, M_PI_2), 1});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(3, 3, M_PI_2), Pose<>(3, 3, 0), 2});

    WeightedGrid2d logical(GRID_SIZE, GRID_SIZE);
    MapStack* mapStack = test::MapStackUtils::grid2d2MapStack(Pose<>::ZERO, 1, &logical);

    Pose<> start = Pose<>(0, 0, 0);
    vector<Pose<>> goals = {
        Pose<>(1, 1, DEG0),
        Pose<>(1, 3, DEG90),
        Pose<>(3, 3, DEG180),
        Pose<>(3, 3, DEG0)
    };

    Solution<Grid2dSolutionItem>* gridSolution = Grid2dSolutionFactory::fromConsecutiveGoals(
        mapStack, start, goals);

    // The solution agrees with itself
    ASSERT_EQ(*gridSolution, *gridSolution) <<
        "The solution is not as expected";

    // The solution agrees with the correct solution
    ASSERT_EQ(*correctSolution, *gridSolution) <<
        "The solution is not as expected";
}
