/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <vector>
#include <ros/ros.h>

using namespace std;

#include <srslib_framework/math/AngleMath.hpp>

#include <srslib_framework/planning/pathplanning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridTrajectoryGenerator.hpp>

#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>

using namespace srs;

TEST(Test_Trajectory, 180ShortTrajectory)
{
    constexpr double DEG0 = 0.0;
    constexpr double DEG90 = AngleMath::deg2rad<double>(-90);
    constexpr double DEG180 = AngleMath::deg2rad<double>(180);

    GridSolutionItem SOLUTION_00 = GridSolutionItem(GridSolutionItem::ROTATE,
        Pose<>(18, 7, DEG180), Pose<>(18, 7, DEG0));

    GridSolutionItem SOLUTION_01 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(18, 7, DEG0), Pose<>(19, 7, DEG0));

    GridSolutionItem SOLUTION_02 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(19, 7, DEG0), Pose<>(20, 7, DEG0));

    GridSolutionItem SOLUTION_03 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(20, 7, DEG0), Pose<>(21, 7, DEG0));

    GridSolutionItem SOLUTION_04 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(21, 7, DEG0), Pose<>(22, 7, DEG0));

    GridSolutionItem SOLUTION_05 = GridSolutionItem(GridSolutionItem::ROTATE,
        Pose<>(22, 7, DEG0), Pose<>(22, 7, DEG90));

    GridSolutionItem SOLUTION_06 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(22, 7, DEG90), Pose<>(23, 7, DEG90));

    GridSolutionItem SOLUTION_07 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(23, 7, DEG90), Pose<>(24, 7, DEG90));

    GridSolutionItem SOLUTION_08 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(24, 7, DEG90), Pose<>(25, 7, DEG90));

    GridSolutionItem SOLUTION_09 = GridSolutionItem(GridSolutionItem::ROTATE,
        Pose<>(25, 7, DEG90), Pose<>(26, 7, DEG0));

    Solution<GridSolutionItem> gridSolution;
    gridSolution.push_back(SOLUTION_00);
    gridSolution.push_back(SOLUTION_01);
    gridSolution.push_back(SOLUTION_02);
    gridSolution.push_back(SOLUTION_03);
    gridSolution.push_back(SOLUTION_04);
    gridSolution.push_back(SOLUTION_05);
    gridSolution.push_back(SOLUTION_06);
    gridSolution.push_back(SOLUTION_07);
    gridSolution.push_back(SOLUTION_08);
    gridSolution.push_back(SOLUTION_09);

    ROS_DEBUG_STREAM(gridSolution);

    Chuck chuck;
    Trajectory<> trajectory;

    GridTrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(gridSolution);
    solutionConverter.getTrajectory(trajectory);

    ROS_DEBUG_STREAM(trajectory);
}
