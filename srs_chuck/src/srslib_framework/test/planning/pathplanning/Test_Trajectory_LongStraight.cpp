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
#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>
#include <srslib_framework/search/SearchPosition.hpp>
using namespace srs;

typedef SolutionNode<Grid2d> SolutionModeType;

TEST(Test_Trajectory, LongStraight)
{
    Grid2d grid(10, 10);

    SolutionModeType SOLUTION_00 = SolutionModeType(SolutionModeType::START,
        Pose<>(0, 0, AngleMath::deg2rad<double>(90)));

    SolutionModeType SOLUTION_01 = SolutionModeType(SolutionModeType::FORWARD,
        Pose<>(1, 0, AngleMath::deg2rad<double>(90)));

    SolutionModeType SOLUTION_02 = SolutionModeType(SolutionModeType::FORWARD,
        Pose<>(2, 0, AngleMath::deg2rad<double>(90)));

    SolutionModeType SOLUTION_03 = SolutionModeType(SolutionModeType::FORWARD,
        Pose<>(3, 0, AngleMath::deg2rad<double>(90)));

    SolutionModeType SOLUTION_04 = SolutionModeType(SolutionModeType::FORWARD,
        Pose<>(4, 0, AngleMath::deg2rad<double>(90)));

    SolutionModeType SOLUTION_05 = SolutionModeType(SolutionModeType::FORWARD,
        Pose<>(5, 0, AngleMath::deg2rad<double>(90)));

    SolutionModeType SOLUTION_06 = SolutionModeType(SolutionModeType::FORWARD,
        Pose<>(6, 0, AngleMath::deg2rad<double>(90)));

    SolutionModeType SOLUTION_07 = SolutionModeType(SolutionModeType::FORWARD,
        Pose<>(7, 0, AngleMath::deg2rad<double>(90)));

    SolutionModeType SOLUTION_08 = SolutionModeType(SolutionModeType::FORWARD,
        Pose<>(8, 0, AngleMath::deg2rad<double>(90)));

    SolutionModeType SOLUTION_09 = SolutionModeType(SolutionModeType::GOAL,
        Pose<>(8, 0, AngleMath::deg2rad<double>(90)));

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

    Chuck chuck;
    Trajectory<> trajectory;

    TrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(solution, 0.1);
    solutionConverter.getTrajectory(trajectory);

    cout << solution << endl;
    cout << trajectory << endl;
}
