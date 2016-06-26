/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
using namespace std;

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/localization/Map.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/planning/pathplanning/TrajectoryGenerator.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>
#include <boost/filesystem.hpp>
#include <ros/ros.h>

using namespace srs;

TEST(Test_Trajectory, Map)
{
    Grid2d::LocationType start(60, 54);
    Grid2d::LocationType goal(95, 54);

	boost::filesystem::path filePath( boost::filesystem::current_path( ) );

    Map* map = new Map();
    map->load("6rhq.yaml");

    AStar<Grid2d>* algorithm = new AStar<Grid2d>(map->getGrid());
    algorithm->search(SearchPosition<Grid2d>(start, 0), SearchPosition<Grid2d>(goal, 0));

    Solution<Grid2d> solution = algorithm->getSolution(map->getResolution());

    ROS_DEBUG_STREAM(solution);

    Chuck chuck;
    Trajectory<> trajectory;

    TrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(solution);
    solutionConverter.getTrajectory(trajectory);

    ROS_DEBUG_STREAM(trajectory);
}
