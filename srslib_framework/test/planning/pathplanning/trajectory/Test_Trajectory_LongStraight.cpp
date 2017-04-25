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
#include <srslib_framework/planning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/trajectory/Grid2dTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/trajectory/Trajectory.hpp>
#include <srslib_framework/robotics/robot_profile/ChuckProfile.hpp>

using namespace srs;

TEST(Test_Trajectory, LongStraight)
{
    constexpr double DEG90 = AngleMath::deg2Rad<double>(90);

    Grid2dSolutionItem SOLUTION_00 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(0, 0, DEG90), Pose<>(0, 1, DEG90));

    Grid2dSolutionItem SOLUTION_01 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(0, 1, DEG90), Pose<>(0, 2, DEG90));

    Grid2dSolutionItem SOLUTION_02 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(0, 2, DEG90), Pose<>(0, 3, DEG90));

    Grid2dSolutionItem SOLUTION_03 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(0, 3, DEG90), Pose<>(0, 4, DEG90));

    Grid2dSolutionItem SOLUTION_04 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(0, 4, DEG90), Pose<>(0, 5, DEG90));

    Grid2dSolutionItem SOLUTION_05 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(0, 5, DEG90), Pose<>(0, 6, DEG90));

    Grid2dSolutionItem SOLUTION_06 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(0, 6, DEG90), Pose<>(0, 7, DEG90));

    Grid2dSolutionItem SOLUTION_07 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(0, 7, DEG90), Pose<>(0, 8, DEG90));

    Grid2dSolutionItem SOLUTION_08 = Grid2dSolutionItem(Grid2dSolutionItem::MOVE,
        Pose<>(0, 8, DEG90), Pose<>(0, 9, DEG90));

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

    ROS_DEBUG_STREAM(gridSolution);

    Chuck chuck;
    Trajectory<> trajectory;

    Grid2dTrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(gridSolution);
    solutionConverter.getTrajectory(trajectory);

    ROS_DEBUG_STREAM(trajectory);
}
