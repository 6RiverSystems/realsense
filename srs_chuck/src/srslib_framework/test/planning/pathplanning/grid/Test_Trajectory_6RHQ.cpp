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
#include <srslib_framework/planning/pathplanning/grid/GridSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/grid/PoseAdapter.hpp>
#include <srslib_framework/robotics/robot_profile/ChuckProfile.hpp>
#include <srslib_framework/search/AStar.hpp>

using namespace srs;

TEST(Test_Trajectory, 6RHQ)
{
    Pose<> robotPose = Pose<>(18.1335, 5.24097, 0.0189141);
    Pose<> goalPose = Pose<>(18.1335, 5.24097, 0);

    // ###FS
//    MapStack* mapStack = MapStackFactory::fromJsonFile("pathplanning/grid/data/6rshq/6rshq.yaml");
//
//    Solution<GridSolutionItem>* gridSolution = GridSolutionFactory::fromGoal(
//        mapStack->getOccupancyMap(), robotPose, goalPose);
//    Solution<GridSolutionItem> gridSolution2 = *gridSolution;
//
//    cout << gridSolution2 << endl;
//
//    Chuck chuck;
//    Trajectory<> trajectory;
//
//    GridTrajectoryGenerator solutionConverter(chuck);
//    solutionConverter.fromSolution(gridSolution2);
//    solutionConverter.getTrajectory(trajectory);
//
//    ROS_DEBUG_STREAM(trajectory);
}
