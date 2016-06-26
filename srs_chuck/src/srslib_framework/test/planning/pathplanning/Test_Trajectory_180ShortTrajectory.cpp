/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <vector>
#include <ros/ros.h>

using namespace std;

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/planning/pathplanning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/TrajectoryGenerator.hpp>
#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>
using namespace srs;

typedef Solution<Grid2d>::NodeType SolutionNodeType;

TEST(Test_Trajectory, 180ShortTrajectory)
{
    constexpr double DEG0 = 0.0;
    constexpr double DEG90 = AngleMath::deg2rad<double>(-90);
    constexpr double DEG180 = AngleMath::deg2rad<double>(180);

    SolutionNodeType SOLUTION_00 = SolutionNodeType(SolutionNodeType::ROTATE,
        Pose<>(18, 7, DEG180), Pose<>(18, 7, DEG0));

    SolutionNodeType SOLUTION_01 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(18, 7, DEG0), Pose<>(19, 7, DEG0));

    SolutionNodeType SOLUTION_02 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(19, 7, DEG0), Pose<>(20, 7, DEG0));

    SolutionNodeType SOLUTION_03 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(20, 7, DEG0), Pose<>(21, 7, DEG0));

    SolutionNodeType SOLUTION_04 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(21, 7, DEG0), Pose<>(22, 7, DEG0));

    SolutionNodeType SOLUTION_05 = SolutionNodeType(SolutionNodeType::ROTATE,
        Pose<>(22, 7, DEG0), Pose<>(22, 7, DEG90));

    SolutionNodeType SOLUTION_06 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(22, 7, DEG90), Pose<>(23, 7, DEG90));

    SolutionNodeType SOLUTION_07 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(23, 7, DEG90), Pose<>(24, 7, DEG90));

    SolutionNodeType SOLUTION_08 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(24, 7, DEG90), Pose<>(25, 7, DEG90));

    SolutionNodeType SOLUTION_09 = SolutionNodeType(SolutionNodeType::ROTATE,
        Pose<>(25, 7, DEG90), Pose<>(26, 7, DEG0));

    Solution<Grid2d> solution;
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

    ROS_DEBUG_STREAM(solution);

    Chuck chuck;
    Trajectory<> trajectory;

    TrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(solution);
    solutionConverter.getTrajectory(trajectory);

    ROS_DEBUG_STREAM(trajectory);
}
