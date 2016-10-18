/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/PoseAdapter.hpp>
#include <srslib_framework/robotics/robot_profile/ChuckProfile.hpp>
#include <srslib_framework/search/AStar.hpp>

#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;


using namespace srs;

// Path not found between Pose {@: 0, x: 18.1335, y: 5.24097, t: 0.0189141} (181,52,0) and
// Pose {@: 1.46645e+09, x: 18.1335, y: 5.24097, t: 0} (181,52,0)

TEST(Test_AStar, SamePlaceOnMap)
{
// ###FS
//    MapStack* mapStack = MapStackFactory::fromJsonFile("data/6rshq/6rshq.yaml");
//    OccupancyMap* map = mapStack->getOccupancyMap();
//
//    // Prepare the start position for the search
//    Pose<> robotPose = Pose<>(18.1335, 5.24097, 0.0189141);
//    Grid2d::Location internalStart;
//    int startAngle;
//    PoseAdapter::pose2Map(robotPose, map, internalStart, startAngle);
//
//    // Prepare the goal position for the search
//    Pose<> goalPose = Pose<>(18.1335, 5.24097, 0);
//    Grid2d::Location internalGoal;
//    int goalAngle;
//    PoseAdapter::pose2Map(goalPose, map, internalGoal, goalAngle);
//
//    test::MemoryWatch memoryWatch;
//
//    AStar<Grid2d>* algorithm = new AStar<Grid2d>(map->getGrid());
//
//    ROS_DEBUG_STREAM("Found: " <<
//        algorithm->search(
//            SearchPosition<Grid2d>(internalStart, startAngle),
//            SearchPosition<Grid2d>(internalGoal, goalAngle)));
//
//    algorithm->clear();
//
//    delete algorithm;
//
//    ROS_DEBUG_STREAM("Memory usage: " << memoryWatch.getMemoryUsage());
//    ROS_DEBUG_STREAM("Memory leaks: " << !memoryWatch.isZero());
}
