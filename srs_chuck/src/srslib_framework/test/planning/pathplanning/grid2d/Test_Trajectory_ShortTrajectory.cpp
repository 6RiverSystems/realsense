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

TEST(Test_Trajectory, ShortTrajectory)
{
    constexpr double DEG90 = AngleMath::deg2Rad<double>(90);
    constexpr double DEG180 = AngleMath::deg2Rad<double>(180);

    Grid2dSolutionItem SOLUTION_00 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(18, 7, DEG90), Pose<>(18, 8, DEG90));

    Grid2dSolutionItem SOLUTION_01 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(18, 8, DEG90), Pose<>(18, 9, DEG90));

    Grid2dSolutionItem SOLUTION_02 = Grid2dSolutionItem(Grid2dSolutionItem::ROTATE,
        Pose<>(18, 9, DEG90), Pose<>(18, 9, DEG180));

    Grid2dSolutionItem SOLUTION_03 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(18, 9, DEG180), Pose<>(17, 9, DEG180));

    Grid2dSolutionItem SOLUTION_04 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(17, 9, DEG180), Pose<>(16, 9, DEG180));

    Solution<Grid2dSolutionItem> solution;
    solution.push_back(SOLUTION_00);
    solution.push_back(SOLUTION_01);
    solution.push_back(SOLUTION_02);
    solution.push_back(SOLUTION_03);
    solution.push_back(SOLUTION_04);

    ROS_DEBUG_STREAM(solution);

    Chuck chuck;
    Trajectory<> trajectory;

    Grid2dTrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(solution);
    solutionConverter.getTrajectory(trajectory);

    ROS_DEBUG_STREAM(trajectory);
}
