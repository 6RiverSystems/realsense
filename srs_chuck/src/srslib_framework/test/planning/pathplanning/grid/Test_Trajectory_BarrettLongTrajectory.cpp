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

#include <srslib_test/planning/localization/MapUtils.hpp>
#include <srslib_test/planning/pathplanning/PathPlanningUtils.hpp>

using namespace srs;

TEST(Test_Trajectory, BarrettLongTrajectory)
{
    Map* map = test::MapUtils::mapFactory("/tmp/srslib_framework/data/barrett.yaml");

    // Looking for a path between
    // Pose {@: 1.4686e+09, x: 14.7855, y: 4.65954, t: 1.53943} (148,47,90)
    // and
    // Pose {@: 1.4686e+09, x: 33.215, y: 51.703, t: 1.5708} (332,522,90)

    Pose<> robotPose = Pose<>(14.7855, 4.65954, 1.53943);
    Pose<> goalPose = Pose<>(33.215, 51.703, 1.5708);
    Solution<GridSolutionItem>* solution = PathPlanningUtils::pose2Solution(map, robotPose, goalPose);
}
