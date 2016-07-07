/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <ros/ros.h>
#include <boost/filesystem.hpp>

#include <srslib_framework/math/AngleMath.hpp>

#include <srslib_framework/localization/Map.hpp>

#include <srslib_framework/planning/pathplanning/grid/GridSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/grid/PoseAdapter.hpp>

#include <srslib_framework/robotics/robot/Chuck.hpp>

#include <srslib_framework/search/AStar.hpp>

using namespace srs;

TEST(Test_Trajectory, Map)
{
    Pose<> robotPose = Pose<>(18.1335, 5.24097, 0.0189141);
    Pose<> goalPose = Pose<>(18.1335, 5.24097, 0);

    boost::filesystem::path filePath = boost::filesystem::canonical(
        "../../../srs_sites/src/srsc_6rhq_rviz/map/6rhq.yaml");
    Map* map = new Map();
    map->load(filePath.generic_string());

    // Prepare the start position for the search
    Grid2d::LocationType internalStart;
    int startAngle;
    PoseAdapter::pose2Map(robotPose, map, internalStart, startAngle);

    // Prepare the goal position for the search
    Grid2d::LocationType internalGoal;
    int goalAngle;
    PoseAdapter::pose2Map(goalPose, map, internalGoal, goalAngle);

    AStar<Grid2d> algorithm(map->getGrid());
    ROS_DEBUG_STREAM("Found: " <<
        algorithm.search(SearchPosition<Grid2d>(internalStart, startAngle),
            SearchPosition<Grid2d>(internalGoal, goalAngle)));

    AStar<Grid2d>::SearchNodeType* solution = algorithm.getSolution();
    Solution<GridSolutionItem>* gridSolution = GridSolutionFactory::fromSearch(solution, map);
    Solution<GridSolutionItem> gridSolution2 = *gridSolution;

    cout << gridSolution2 << endl;

    Chuck chuck;
    Trajectory<> trajectory;

    GridTrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(gridSolution2);
    solutionConverter.getTrajectory(trajectory);

    ROS_DEBUG_STREAM(trajectory);
}
