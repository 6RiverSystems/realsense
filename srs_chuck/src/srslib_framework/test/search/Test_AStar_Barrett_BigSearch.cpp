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

using namespace srs;

TEST(Test_AStar, Barrett_BigSearch)
{
    // ###FS
//    MapStack* mapStack = MapStackFactory::fromJsonFile("pathplanning/grid2d/data/barrett/barrett.yaml");
//
//    // Looking for a path between
//    // Pose {@: 1.4686e+09, x: 14.7855, y: 4.65954, t: 1.53943} (148,47,90)
//    // and
//    // Pose {@: 1.4686e+09, x: 33.215, y: 51.703, t: 1.5708} (332,522,90)
//
//    Pose<> robotPose = Pose<>(14.7855, 4.65954, 1.53943);
//    Pose<> goalPose = Pose<>(33.215, 51.703, 1.5708);
//    Solution<Grid2dSolutionItem>* solution = Grid2dSolutionFactory::fromGoal(
//        mapStack->getOccupancyMap(), robotPose, goalPose);
}
