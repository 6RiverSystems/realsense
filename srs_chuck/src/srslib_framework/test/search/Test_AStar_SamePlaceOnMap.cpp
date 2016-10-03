/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <ros/ros.h>

// #include <srslib_framework/localization/Map.hpp>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/grid/PoseAdapter.hpp>
#include <srslib_framework/robotics/robot_profile/ChuckProfile.hpp>
#include <srslib_framework/search/AStar.hpp>

#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;


using namespace srs;

// Path not found between Pose {@: 0, x: 18.1335, y: 5.24097, t: 0.0189141} (181,52,0) and
// Pose {@: 1.46645e+09, x: 18.1335, y: 5.24097, t: 0} (181,52,0)

TEST(Test_AStar, SamePlaceOnMap)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("data/6rshq/6rshq.yaml");
    OccupancyMap* map = mapStack->getOccupancyMap();

    // Prepare the start position for the search
    Pose<> robotPose = Pose<>(18.1335, 5.24097, 0.0189141);
    Grid2d::LocationType internalStart;
    int startAngle;
    PoseAdapter::pose2Map(robotPose, map, internalStart, startAngle);

    // Prepare the goal position for the search
    Pose<> goalPose = Pose<>(18.1335, 5.24097, 0);
    Grid2d::LocationType internalGoal;
    int goalAngle;
    PoseAdapter::pose2Map(goalPose, map, internalGoal, goalAngle);

    test::MemoryWatch memoryWatch;

    AStar<Grid2d>* algorithm = new AStar<Grid2d>(map->getGrid());

    ROS_DEBUG_STREAM("Found: " <<
        algorithm->search(
            SearchPosition<Grid2d>(internalStart, startAngle),
            SearchPosition<Grid2d>(internalGoal, goalAngle)));

    algorithm->clear();

    delete algorithm;

    ROS_DEBUG_STREAM("Memory usage: " << memoryWatch.getMemoryUsage());
    ROS_DEBUG_STREAM("Memory leaks: " << !memoryWatch.isZero());

// ###FS
//    Pose<> robotPose = Pose<>(18.1335, 5.24097, 0.0189141);
//    Pose<> goalPose = Pose<>(18.1335, 5.24097, 0);
//
//    Map* map = new Map();
//    map->load("/tmp/srslib_framework/data/6rhq.yaml");
//
//    // Prepare the start position for the search
//    Grid2d::LocationType internalStart;
//    int startAngle;
//    PoseAdapter::pose2Map(robotPose, map, internalStart, startAngle);
//
//    // Prepare the goal position for the search
//    Grid2d::LocationType internalGoal;
//    int goalAngle;
//    PoseAdapter::pose2Map(goalPose, map, internalGoal, goalAngle);
//
//    AStar<Grid2d>* algorithm = new AStar<Grid2d>(map->getGrid());
//    ROS_DEBUG_STREAM("Found: " <<
//        algorithm->search(SearchPosition<Grid2d>(internalStart, startAngle),
//            SearchPosition<Grid2d>(internalGoal, goalAngle)));
}
