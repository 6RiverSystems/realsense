/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
using namespace std;

#include <srslib_framework/math/AngleMath.hpp>

#include <srslib_framework/planning/pathplanning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>

#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>

using namespace srs;

TEST(Test_Trajectory, SingleRotation)
{
    constexpr double DEG90 = AngleMath::deg2Rad<double>(90);
    constexpr double DEG180 = AngleMath::deg2Rad<double>(180);

    GridSolutionItem SOLUTION_00 = GridSolutionItem(GridSolutionItem::ROTATE,
        Pose<>(18, 9, DEG90), Pose<>(18, 9, DEG180));

    Solution<GridSolutionItem> gridSolution;
    gridSolution.push_back(SOLUTION_00);

    ROS_DEBUG_STREAM(gridSolution);

    Chuck chuck;
    Trajectory<> trajectory;

    GridTrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(gridSolution);
    solutionConverter.getTrajectory(trajectory);

    ROS_DEBUG_STREAM(trajectory);
}
