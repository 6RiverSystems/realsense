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
#include <srslib_framework/robotics/robot_profile/ChuckProfile.hpp>
using namespace srs;

TEST(Test_Trajectory, LongTrajectory)
{
    constexpr double DEG0 = 0.0;
    constexpr double DEG90 = AngleMath::deg2Rad<double>(90);

    GridSolutionItem SOLUTION_00 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(0, 0, DEG0), Pose<>(1, 0, DEG0));

    GridSolutionItem SOLUTION_01 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(1, 0, DEG0), Pose<>(2, 0, DEG0));

    GridSolutionItem SOLUTION_02 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(2, 0, DEG0), Pose<>(3, 0, DEG0));

    GridSolutionItem SOLUTION_03 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(3, 0, DEG0), Pose<>(4, 0, DEG0));

    GridSolutionItem SOLUTION_04 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(4, 0, DEG0), Pose<>(5, 0, DEG0));

    GridSolutionItem SOLUTION_05 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(5, 0, DEG0), Pose<>(6, 0, DEG0));

    GridSolutionItem SOLUTION_06 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(6, 0, DEG0), Pose<>(7, 0, DEG0));

    GridSolutionItem SOLUTION_07 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(7, 0, DEG0), Pose<>(8, 0, DEG0));

    GridSolutionItem SOLUTION_08 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(8, 0, DEG0), Pose<>(9, 0, DEG0));

    GridSolutionItem SOLUTION_09 = GridSolutionItem(GridSolutionItem::ROTATE,
        Pose<>(9, 0, DEG0), Pose<>(9, 0, DEG90));

    GridSolutionItem SOLUTION_10 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(9, 0, DEG90), Pose<>(9, 1, DEG90));

    GridSolutionItem SOLUTION_11 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(9, 1, DEG90), Pose<>(9, 2, DEG90));

    GridSolutionItem SOLUTION_12 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(9, 2, DEG90), Pose<>(9, 3, DEG90));

    GridSolutionItem SOLUTION_13 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(9, 3, DEG90), Pose<>(9, 4, DEG90));

    GridSolutionItem SOLUTION_14 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(9, 4, DEG90), Pose<>(9, 5, DEG90));

    GridSolutionItem SOLUTION_15 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(9, 5, DEG90), Pose<>(9, 6, DEG90));

    Solution<GridSolutionItem> solution;
    solution.push_back(SOLUTION_00);
    solution.push_back(SOLUTION_01);
    solution.push_back(SOLUTION_02);
    solution.push_back(SOLUTION_03);
    solution.push_back(SOLUTION_04);
    solution.push_back(SOLUTION_05);
    solution.push_back(SOLUTION_06);
    solution.push_back(SOLUTION_07);
    solution.push_back(SOLUTION_08);
    solution.push_back(SOLUTION_09);
    solution.push_back(SOLUTION_10);
    solution.push_back(SOLUTION_11);
    solution.push_back(SOLUTION_12);
    solution.push_back(SOLUTION_13);
    solution.push_back(SOLUTION_14);
    solution.push_back(SOLUTION_15);

    ROS_DEBUG_STREAM(solution);

    Chuck chuck;
    Trajectory<> trajectory;

    GridTrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(solution);
    solutionConverter.getTrajectory(trajectory);

    ROS_DEBUG_STREAM(trajectory);
}
