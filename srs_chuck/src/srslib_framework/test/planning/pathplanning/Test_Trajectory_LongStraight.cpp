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

typedef Solution<Grid2d>::NodeType SolutionNodeType;

TEST(Test_Trajectory, LongStraight)
{
    constexpr double DEG90 = AngleMath::deg2rad<double>(90);

    SolutionNodeType SOLUTION_00 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(0, 0, DEG90), Pose<>(0, 1, DEG90));

    SolutionNodeType SOLUTION_01 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(0, 1, DEG90), Pose<>(0, 2, DEG90));

    SolutionNodeType SOLUTION_02 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(0, 2, DEG90), Pose<>(0, 3, DEG90));

    SolutionNodeType SOLUTION_03 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(0, 3, DEG90), Pose<>(0, 4, DEG90));

    SolutionNodeType SOLUTION_04 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(0, 4, DEG90), Pose<>(0, 5, DEG90));

    SolutionNodeType SOLUTION_05 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(0, 5, DEG90), Pose<>(0, 6, DEG90));

    SolutionNodeType SOLUTION_06 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(0, 6, DEG90), Pose<>(0, 7, DEG90));

    SolutionNodeType SOLUTION_07 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(0, 7, DEG90), Pose<>(0, 8, DEG90));

    SolutionNodeType SOLUTION_08 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(0, 8, DEG90), Pose<>(0, 9, DEG90));

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

    cout << solution << endl;

    Chuck chuck;
    Trajectory<> trajectory;

    TrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(solution, 1.0 / 100.0);
    solutionConverter.getTrajectory(trajectory);

    cout << trajectory << endl;
}
