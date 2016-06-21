/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/planning/pathplanning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/TrajectoryGenerator.hpp>
#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>
using namespace srs;

typedef SolutionNode<Grid2d> SolutionNodeType;

TEST(Test_Trajectory, LongTrajectory)
{
    constexpr double DEG0 = 0.0;
    constexpr double DEG90 = AngleMath::deg2rad<double>(90);

    SolutionNodeType SOLUTION_00 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(0, 0, DEG0), Pose<>(1, 0, DEG0));

    SolutionNodeType SOLUTION_01 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(1, 0, DEG0), Pose<>(2, 0, DEG0));

    SolutionNodeType SOLUTION_02 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(2, 0, DEG0), Pose<>(3, 0, DEG0));

    SolutionNodeType SOLUTION_03 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(3, 0, DEG0), Pose<>(4, 0, DEG0));

    SolutionNodeType SOLUTION_04 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(4, 0, DEG0), Pose<>(5, 0, DEG0));

    SolutionNodeType SOLUTION_05 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(5, 0, DEG0), Pose<>(6, 0, DEG0));

    SolutionNodeType SOLUTION_06 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(6, 0, DEG0), Pose<>(7, 0, DEG0));

    SolutionNodeType SOLUTION_07 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(7, 0, DEG0), Pose<>(8, 0, DEG0));

    SolutionNodeType SOLUTION_08 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(8, 0, DEG0), Pose<>(9, 0, DEG0));

    SolutionNodeType SOLUTION_09 = SolutionNodeType(SolutionNodeType::ROTATE,
        Pose<>(9, 0, DEG0), Pose<>(9, 0, DEG90));

    SolutionNodeType SOLUTION_10 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(9, 0, DEG90), Pose<>(9, 1, DEG90));

    SolutionNodeType SOLUTION_11 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(9, 1, DEG90), Pose<>(9, 2, DEG90));

    SolutionNodeType SOLUTION_12 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(9, 2, DEG90), Pose<>(9, 3, DEG90));

    SolutionNodeType SOLUTION_13 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(9, 3, DEG90), Pose<>(9, 4, DEG90));

    SolutionNodeType SOLUTION_14 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(9, 4, DEG90), Pose<>(9, 5, DEG90));

    SolutionNodeType SOLUTION_15 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(9, 5, DEG90), Pose<>(9, 6, DEG90));

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
    solution.push_back(SOLUTION_10);
    solution.push_back(SOLUTION_11);
    solution.push_back(SOLUTION_12);
    solution.push_back(SOLUTION_13);
    solution.push_back(SOLUTION_14);
    solution.push_back(SOLUTION_15);

    cout << solution << endl;

    Chuck chuck;
    Trajectory<> trajectory;

    TrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(solution, 1.0 / 100.0);
    solutionConverter.getTrajectory(trajectory);

    cout << trajectory << endl;
}
