/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/localization/Map.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/planning/pathplanning/TrajectoryGenerator.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>

using namespace srs;

TEST(Test_Trajectory, Map)
{
    Grid2d::LocationType start(60, 54);
    Grid2d::LocationType goal(95, 54);

    Map* map = new Map();
    map->load("/home/fsantini/projects/repos/ros/srs_sites/src/srsc_6rhq/map/6rhq.yaml");

    AStar<Grid2d>* algorithm = new AStar<Grid2d>(map->getGrid());
    algorithm->search(SearchPosition<Grid2d>(start, 0), SearchPosition<Grid2d>(goal, 0));

    Solution<Grid2d> solution = algorithm->getSolution(map->getResolution());
    cout << solution << endl;

    Chuck chuck;

    // Calculate the trajectory
    Trajectory<> trajectory;
    TrajectoryGenerator converter(chuck);

    converter.fromSolution(solution, 1.0 / 100.0);
    converter.getTrajectory(trajectory);

    cout << trajectory << endl;
}
