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
#include <srslib_framework/search/SearchPosition.hpp>
using namespace srs;

typedef SolutionNode<Grid2d> SolutionNodeType;

TEST(Test_Trajectory, LongTrajectory)
{
    SolutionNodeType SOLUTION_00 = SolutionNodeType(SolutionNodeType::START,
        Pose<>(0, 0, 0));

    SolutionNodeType SOLUTION_01 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(1, 0, 0));

    SolutionNodeType SOLUTION_02 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(2, 0, 0));

    SolutionNodeType SOLUTION_03 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(3, 0, 0));

    SolutionNodeType SOLUTION_04 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(4, 0, 0));

    SolutionNodeType SOLUTION_05 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(5, 0, 0));

    SolutionNodeType SOLUTION_06 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(6, 0, 0));

    SolutionNodeType SOLUTION_07 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(7, 0, 0));

    SolutionNodeType SOLUTION_08 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(8, 0, 0));

    SolutionNodeType SOLUTION_09 = SolutionNodeType(SolutionNodeType::ROTATE_P90,
        Pose<>(8, 0, AngleMath::deg2rad<double>(90)));

    SolutionNodeType SOLUTION_10 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(8, 1, AngleMath::deg2rad<double>(90)));

    SolutionNodeType SOLUTION_11 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(8, 2, AngleMath::deg2rad<double>(90)));

    SolutionNodeType SOLUTION_12 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(8, 3, AngleMath::deg2rad<double>(90)));

    SolutionNodeType SOLUTION_13 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(8, 4, AngleMath::deg2rad<double>(90)));

    SolutionNodeType SOLUTION_14 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(8, 5, AngleMath::deg2rad<double>(90)));

    SolutionNodeType SOLUTION_15 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(8, 6, AngleMath::deg2rad<double>(90)));

    SolutionNodeType SOLUTION_16 = SolutionNodeType(SolutionNodeType::GOAL,
        Pose<>(8, 7, AngleMath::deg2rad<double>(90)));

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
    solution.push_back(SOLUTION_16);

    Chuck chuck;
    Trajectory<> trajectory;

    TrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(solution, 0.1);
    solutionConverter.getTrajectory(trajectory);

    cout << solution << endl;
    cout << trajectory << endl;
}
