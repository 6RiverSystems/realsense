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

TEST(Test_Trajectory, ShortStraight)
{
    Grid2dSolutionItem SOLUTION_00 = Grid2dSolutionItem(
        Grid2dSolutionItem::MOVE,
        Pose<>(3, 3, 0), Pose<>(3, 4, 0));

    Solution<Grid2dSolutionItem>* solution = Solution<Grid2dSolutionItem>::instanceOfValid(SOLUTION_00);

    ROS_DEBUG_STREAM(*solution);

    Chuck chuck;
    Trajectory<> trajectory;

    Grid2dTrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(solution);
    solutionConverter.getTrajectory(trajectory);

    ROS_DEBUG_STREAM(trajectory);
}
