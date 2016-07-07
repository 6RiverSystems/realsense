/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
using namespace std;

#include <srslib_framework/localization/Map.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/localization/MapNote.hpp>
using namespace srs;

// Path not found between Pose {@: 0, x: 18.1335, y: 5.24097, t: 0.0189141} (181,52,0) and
// Pose {@: 1.46645e+09, x: 18.1335, y: 5.24097, t: 0} (181,52,0)

TEST(Test_AStar, SamePlaceOnMap)
{
    Pose<> robotPose = Pose<>(18.1335, 5.24097, 0.0189141);
    Pose<> goalPose = Pose<>(18.1335, 5.24097, 0);

    Map* map = new Map();
    map->load("6rhq.yaml");

    // Prepare the start position for the search
    int fromR = 0;
    int fromC = 0;
    map->getMapCoordinates(robotPose.x, robotPose.y, fromC, fromR);
    Grid2d::LocationType internalStart(fromC, fromR);
    int startAngle = AngleMath::normalizeRad2deg90(robotPose.theta);

    // Prepare the goal position for the search
    int toR = 0;
    int toC = 0;
    map->getMapCoordinates(goalPose.x, goalPose.y, toC, toR);
    Grid2d::LocationType internalGoal(toC, toR);
    int goalAngle = AngleMath::normalizeRad2deg90(goalPose.theta);

    AStar<Grid2d>* algorithm = new AStar<Grid2d>(map->getGrid());
    ROS_DEBUG_STREAM("Found: " <<
        algorithm->search(SearchPosition<Grid2d>(internalStart, startAngle),
            SearchPosition<Grid2d>(internalGoal, goalAngle)));
}
