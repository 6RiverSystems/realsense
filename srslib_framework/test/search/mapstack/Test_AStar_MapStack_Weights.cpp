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

#include <srslib_framework/datastructure/Location.hpp>
#include <srslib_framework/datastructure/Position.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapAdapter.hpp>
#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>
#include <srslib_framework/planning/Solution.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/Plan.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackSingleGoal.hpp>

#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>
#include <srslib_test/localization/map/MapStackUtils.hpp>
#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;

constexpr int GRID_SIZE = 5;

TEST(Test_AStar_MapStack_Weights, SmallSearchWithWeights)
{
    Solution<Grid2dSolutionItem>* correctSolution = Solution<Grid2dSolutionItem>::instanceOfValidEmpty();
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(3, 1, 0), Pose<>(3, 1, -M_PI_2), 1});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(3, 1, -M_PI_2), Pose<>(3, 0, -M_PI_2), 2});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(3, 0, -M_PI_2), Pose<>(3, 0, -M_PI), 3});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(3, 0, -M_PI), Pose<>(2, 0, -M_PI), 4});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(2, 0, -M_PI), Pose<>(1, 0, -M_PI), 5});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(1, 0, -M_PI), Pose<>(1, 0, M_PI_2), 6});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(1, 0, M_PI_2), Pose<>(1, 1, M_PI_2), 7});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(1, 1, M_PI_2), Pose<>(1, 1, 0), 8});

    WeightedGrid2d logical(GRID_SIZE, GRID_SIZE);
    logical.setWeights(Location(2, 1), 0, 0, 0, 0, 0, 100, 100, 100);
    logical.setWeights(Location(3, 1), 0, 0, 0, 0, 0, 100, 100, 100);

    MapStack* mapStack = test::MapStackUtils::grid2d2MapStack(Pose<>::ZERO, 1, &logical);

    Position start(3, 1, 0);
    Position goal(1, 1, 0);

    Solution<Grid2dSolutionItem>* gridSolution = Grid2dSolutionFactory::fromSingleGoal(
        mapStack, start, goal);

    // The solution agrees with itself
    ASSERT_EQ(*gridSolution, *gridSolution) <<
        "The solution is not as expected";

    // The solution agrees with the correct solution
    ASSERT_EQ(*correctSolution, *gridSolution) <<
        "The solution is not as expected";
}

TEST(Test_AStar_MapStack_Weights, WeightsAndThisWalls)
{
    Solution<Grid2dSolutionItem>* correctSolution = Solution<Grid2dSolutionItem>::instanceOfValidEmpty();
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(3, 1, 0), Pose<>(3, 1, -M_PI_2), 1});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(3, 1, -M_PI_2), Pose<>(3, 1, -M_PI), 2});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(3, 1, -M_PI), Pose<>(2, 1, -M_PI), 103});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(2, 1, -M_PI), Pose<>(2, 1, M_PI_2), 104});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(2, 1, M_PI_2), Pose<>(2, 2, M_PI_2), 105});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(2, 2, M_PI_2), Pose<>(2, 2, -M_PI), 106});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(2, 2, -M_PI), Pose<>(1, 2, -M_PI), 107});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(1, 2, -M_PI), Pose<>(1, 2, -M_PI_2), 108});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(1, 2, -M_PI_2), Pose<>(1, 1, -M_PI_2), 109});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(1, 1, -M_PI_2), Pose<>(1, 1, 0), 110});

    WeightedGrid2d logical(GRID_SIZE, GRID_SIZE);
    logical.setWeights(Location(2, 1), 0, 0, 0, 0, 0, 100, 100, 100);
    logical.setWeights(Location(3, 1), 0, 0, 0, 0, 0, 100, 100, 100);
    test::Grid2dUtils::addObstacle(logical, 3, 0, 3, 0);
    test::Grid2dUtils::addObstacle(logical, 3, 2, 3, 4);

    MapStack* mapStack = test::MapStackUtils::grid2d2MapStack(Pose<>::ZERO, 1, &logical);

    Position start(Location(3, 1), 0);
    Position goal(Location(1, 1), 0);

    Solution<Grid2dSolutionItem>* gridSolution = Grid2dSolutionFactory::fromSingleGoal(
        mapStack, start, goal);

    // The solution agrees with itself
    ASSERT_EQ(*gridSolution, *gridSolution) <<
        "The solution does not agree with itself";

    // The solution agrees with the correct solution
    ASSERT_EQ(*correctSolution, *gridSolution) <<
        "The solution is not as expected";
}

TEST(Test_AStar_MapStack_Weights, WeightsAndThickWalls)
{
    Solution<Grid2dSolutionItem>* correctSolution = Solution<Grid2dSolutionItem>::instanceOfValidEmpty();
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(3, 1, 0), Pose<>(3, 1, -M_PI_2), 1});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(3, 1, -M_PI_2), Pose<>(3, 1, -M_PI), 2});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(3, 1, -M_PI), Pose<>(2, 1, -M_PI), 103});
    correctSolution->push_back({Grid2dSolutionItem::MOVE, Pose<>(2, 1, -M_PI), Pose<>(1, 1, -M_PI), 204});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(1, 1, -M_PI), Pose<>(1, 1, M_PI_2), 205});
    correctSolution->push_back({Grid2dSolutionItem::ROTATE, Pose<>(1, 1, M_PI_2), Pose<>(1, 1, 0), 206});

    WeightedGrid2d logical(GRID_SIZE, GRID_SIZE);
    logical.setWeights(Location(2, 1), 0, 0, 0, 0, 0, 100, 100, 100);
    logical.setWeights(Location(3, 1), 0, 0, 0, 0, 0, 100, 100, 100);
    test::Grid2dUtils::addObstacle(logical, 2, 0, 2, 0);
    test::Grid2dUtils::addObstacle(logical, 2, 2, 2, 4);
    test::Grid2dUtils::addObstacle(logical, 3, 0, 3, 0);
    test::Grid2dUtils::addObstacle(logical, 3, 2, 3, 4);

    MapStack* mapStack = test::MapStackUtils::grid2d2MapStack(Pose<>::ZERO, 1, &logical);

    Position start(Location(3, 1), 0);
    Position goal(Location(1, 1), 0);

    Solution<Grid2dSolutionItem>* gridSolution = Grid2dSolutionFactory::fromSingleGoal(
        mapStack, start, goal);

    // The solution agrees with itself
    ASSERT_EQ(*gridSolution, *gridSolution) <<
        "The solution does not agree with itself";

    // The solution agrees with the correct solution
    ASSERT_EQ(*correctSolution, *gridSolution) <<
        "The solution is not as expected";
}
