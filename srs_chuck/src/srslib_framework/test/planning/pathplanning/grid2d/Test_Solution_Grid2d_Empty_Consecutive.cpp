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

using namespace srs;

static constexpr double DEG0 = 0.0;
static constexpr double DEG90 = AngleMath::deg2Rad<double>(90);
static constexpr double DEG180 = AngleMath::deg2Rad<double>(180);
static constexpr double DEG270 = AngleMath::deg2Rad<double>(270);

TEST(Test_Solution_Grid2d, Empty_Consecutive)
{
    Pose<> robotPose = Pose<>(3, 3, DEG0);
    vector<Pose<>> goals = {
        Pose<>(4, 3, DEG0),
        Pose<>(4, 4, DEG90),
        Pose<>(3, 4, DEG180),
        Pose<>(3, 3, DEG270)
    };

    MapStack* mapStack = MapStackFactory::fromJsonFile("pathplanning/data/empty/empty.yaml");

    Solution<Grid2dSolutionItem>* gridSolution = Grid2dSolutionFactory::fromConsecutiveGoals(
        mapStack->getOccupancyMap(), robotPose, goals);
}
