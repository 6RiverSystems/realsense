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
#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridTrajectoryGenerator.hpp>

#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>
using namespace srs;

TEST(Test_Trajectory, ShortTrajectory)
{
    constexpr double DEG90 = AngleMath::deg2rad<double>(90);
    constexpr double DEG180 = AngleMath::deg2rad<double>(180);

    GridSolutionItem SOLUTION_00 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(18, 7, DEG90), Pose<>(18, 8, DEG90));

    GridSolutionItem SOLUTION_01 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(18, 8, DEG90), Pose<>(18, 9, DEG90));

    GridSolutionItem SOLUTION_02 = GridSolutionItem(GridSolutionItem::ROTATE,
        Pose<>(18, 9, DEG90), Pose<>(18, 9, DEG180));

    GridSolutionItem SOLUTION_03 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(18, 9, DEG180), Pose<>(17, 9, DEG180));

    GridSolutionItem SOLUTION_04 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(17, 9, DEG180), Pose<>(16, 9, DEG180));

    Solution<GridSolutionItem> solution;
    solution.push_back(SOLUTION_00);
    solution.push_back(SOLUTION_01);
    solution.push_back(SOLUTION_02);
    solution.push_back(SOLUTION_03);
    solution.push_back(SOLUTION_04);

    ROS_DEBUG_STREAM(solution);

    Chuck chuck;
    Trajectory<> trajectory;

    GridTrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(solution);
    solutionConverter.getTrajectory(trajectory);

    ROS_DEBUG_STREAM(trajectory);
}
