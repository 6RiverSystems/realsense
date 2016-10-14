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
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dTrajectoryGenerator.hpp>

#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/robot_profile/ChuckProfile.hpp>
using namespace srs;

TEST(Test_Trajectory, LongTrajectory)
{
    constexpr double DEG0 = 0.0;
    constexpr double DEG90 = AngleMath::deg2Rad<double>(90);

    Grid2dSolutionItem SOLUTION_00 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(0, 0, DEG0), Pose<>(1, 0, DEG0));

    Grid2dSolutionItem SOLUTION_01 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(1, 0, DEG0), Pose<>(2, 0, DEG0));

    Grid2dSolutionItem SOLUTION_02 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(2, 0, DEG0), Pose<>(3, 0, DEG0));

    Grid2dSolutionItem SOLUTION_03 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(3, 0, DEG0), Pose<>(4, 0, DEG0));

    Grid2dSolutionItem SOLUTION_04 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(4, 0, DEG0), Pose<>(5, 0, DEG0));

    Grid2dSolutionItem SOLUTION_05 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(5, 0, DEG0), Pose<>(6, 0, DEG0));

    Grid2dSolutionItem SOLUTION_06 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(6, 0, DEG0), Pose<>(7, 0, DEG0));

    Grid2dSolutionItem SOLUTION_07 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(7, 0, DEG0), Pose<>(8, 0, DEG0));

    Grid2dSolutionItem SOLUTION_08 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(8, 0, DEG0), Pose<>(9, 0, DEG0));

    Grid2dSolutionItem SOLUTION_09 = Grid2dSolutionItem(Grid2dSolutionItem::ROTATE,
        Pose<>(9, 0, DEG0), Pose<>(9, 0, DEG90));

    Grid2dSolutionItem SOLUTION_10 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(9, 0, DEG90), Pose<>(9, 1, DEG90));

    Grid2dSolutionItem SOLUTION_11 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(9, 1, DEG90), Pose<>(9, 2, DEG90));

    Grid2dSolutionItem SOLUTION_12 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(9, 2, DEG90), Pose<>(9, 3, DEG90));

    Grid2dSolutionItem SOLUTION_13 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(9, 3, DEG90), Pose<>(9, 4, DEG90));

    Grid2dSolutionItem SOLUTION_14 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(9, 4, DEG90), Pose<>(9, 5, DEG90));

    Grid2dSolutionItem SOLUTION_15 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(9, 5, DEG90), Pose<>(9, 6, DEG90));

    Solution<Grid2dSolutionItem> solution;
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

    Grid2dTrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(solution);
    solutionConverter.getTrajectory(trajectory);

    ROS_DEBUG_STREAM(trajectory);
}
