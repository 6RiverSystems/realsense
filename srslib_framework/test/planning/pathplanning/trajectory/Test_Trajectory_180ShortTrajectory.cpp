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
#include <srslib_framework/planning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/trajectory/Grid2dTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/trajectory/Trajectory.hpp>
#include <srslib_framework/robotics/robot_profile/ChuckProfile.hpp>

using namespace srs;

TEST(Test_Trajectory, 180ShortTrajectory)
{
    constexpr double DEG0 = 0.0;
    constexpr double DEG90 = AngleMath::deg2Rad<double>(-90);
    constexpr double DEG180 = AngleMath::deg2Rad<double>(180);

    Grid2dSolutionItem SOLUTION_00 = Grid2dSolutionItem(Grid2dSolutionItem::ROTATE,
        Pose<>(18, 7, DEG180), Pose<>(18, 7, DEG0));

    Grid2dSolutionItem SOLUTION_01 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(18, 7, DEG0), Pose<>(19, 7, DEG0));

    Grid2dSolutionItem SOLUTION_02 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(19, 7, DEG0), Pose<>(20, 7, DEG0));

    Grid2dSolutionItem SOLUTION_03 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(20, 7, DEG0), Pose<>(21, 7, DEG0));

    Grid2dSolutionItem SOLUTION_04 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(21, 7, DEG0), Pose<>(22, 7, DEG0));

    Grid2dSolutionItem SOLUTION_05 = Grid2dSolutionItem(Grid2dSolutionItem::ROTATE,
        Pose<>(22, 7, DEG0), Pose<>(22, 7, DEG90));

    Grid2dSolutionItem SOLUTION_06 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(22, 7, DEG90), Pose<>(23, 7, DEG90));

    Grid2dSolutionItem SOLUTION_07 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(23, 7, DEG90), Pose<>(24, 7, DEG90));

    Grid2dSolutionItem SOLUTION_08 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(24, 7, DEG90), Pose<>(25, 7, DEG90));

    Grid2dSolutionItem SOLUTION_09 = Grid2dSolutionItem(Grid2dSolutionItem::ROTATE,
        Pose<>(25, 7, DEG90), Pose<>(26, 7, DEG0));

    Solution<Grid2dSolutionItem>* gridSolution = Solution<Grid2dSolutionItem>::instanceOfValidEmpty();
    gridSolution->push_back(SOLUTION_00);
    gridSolution->push_back(SOLUTION_01);
    gridSolution->push_back(SOLUTION_02);
    gridSolution->push_back(SOLUTION_03);
    gridSolution->push_back(SOLUTION_04);
    gridSolution->push_back(SOLUTION_05);
    gridSolution->push_back(SOLUTION_06);
    gridSolution->push_back(SOLUTION_07);
    gridSolution->push_back(SOLUTION_08);
    gridSolution->push_back(SOLUTION_09);

    ROS_DEBUG_STREAM(gridSolution);

    Chuck chuck;
    Trajectory<> trajectory;

    Grid2dTrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(gridSolution);
    solutionConverter.getTrajectory(trajectory);

    ROS_DEBUG_STREAM(trajectory);
}
