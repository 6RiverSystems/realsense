/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
using namespace std;

#include <boost/filesystem.hpp>
#include <ros/ros.h>

#include <srslib_framework/localization/Map.hpp>

#include <srslib_framework/planning/pathplanning/grid/GridSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridTrajectoryGenerator.hpp>

#include <srslib_framework/robotics/robot/Chuck.hpp>

#include <srslib_framework/search/AStar.hpp>

using namespace srs;

TEST(Test_Trajectory, Map)
{
    Grid2d::LocationType start(60, 54);
    Grid2d::LocationType goal(95, 54);

    boost::filesystem::path filePath = boost::filesystem::canonical(
        "../../../srs_sites/src/srsc_6rhq/map/6rhq.yaml");

    Map* map = new Map();
    map->load(filePath.generic_string());

    AStar<Grid2d>* algorithm = new AStar<Grid2d>(map->getGrid());
    algorithm->search(SearchPosition<Grid2d>(start, 0), SearchPosition<Grid2d>(goal, 0));

    SearchNode<Grid2d>* solution = algorithm->getSolution();
    Solution<GridSolutionItem>* gridSolution = GridSolutionFactory::fromSearch(solution, map);

    ROS_DEBUG_STREAM(*gridSolution);

    Chuck chuck;
    Trajectory<> trajectory;

    GridTrajectoryGenerator solutionConverter(chuck);

    Solution<GridSolutionItem> gridSolution2 = *gridSolution;
    solutionConverter.fromSolution(gridSolution2);
    solutionConverter.getTrajectory(trajectory);

    ROS_DEBUG_STREAM(trajectory);
}
