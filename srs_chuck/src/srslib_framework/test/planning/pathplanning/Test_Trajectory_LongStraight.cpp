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

TEST(Test_Trajectory, LongStraight)
{
    constexpr double DEG90 = AngleMath::deg2rad<double>(90);

    GridSolutionItem SOLUTION_00 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(0, 0, DEG90), Pose<>(0, 1, DEG90));

    GridSolutionItem SOLUTION_01 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(0, 1, DEG90), Pose<>(0, 2, DEG90));

    GridSolutionItem SOLUTION_02 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(0, 2, DEG90), Pose<>(0, 3, DEG90));

    GridSolutionItem SOLUTION_03 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(0, 3, DEG90), Pose<>(0, 4, DEG90));

    GridSolutionItem SOLUTION_04 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(0, 4, DEG90), Pose<>(0, 5, DEG90));

    GridSolutionItem SOLUTION_05 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(0, 5, DEG90), Pose<>(0, 6, DEG90));

    GridSolutionItem SOLUTION_06 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(0, 6, DEG90), Pose<>(0, 7, DEG90));

    GridSolutionItem SOLUTION_07 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(0, 7, DEG90), Pose<>(0, 8, DEG90));

    GridSolutionItem SOLUTION_08 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(0, 8, DEG90), Pose<>(0, 9, DEG90));

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

    ROS_DEBUG_STREAM(gridSolution);

    Chuck chuck;
    Trajectory<> trajectory;

    GridTrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(gridSolution);
    solutionConverter.getTrajectory(trajectory);

    ROS_DEBUG_STREAM(trajectory);
}
