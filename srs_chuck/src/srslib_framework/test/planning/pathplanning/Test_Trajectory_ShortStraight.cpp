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

TEST(Test_Trajectory, ShortStraight)
{
    GridSolutionItem SOLUTION_00 = GridSolutionItem(GridSolutionItem::MOVE,
        Pose<>(3, 3, 0), Pose<>(3, 4, 0));

    Solution<GridSolutionItem> solution;
    solution.push_back(SOLUTION_00);

    ROS_DEBUG_STREAM(solution);

    Chuck chuck;
    Trajectory<> trajectory;

    GridTrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(solution);
    solutionConverter.getTrajectory(trajectory);

    ROS_DEBUG_STREAM(trajectory);
}
