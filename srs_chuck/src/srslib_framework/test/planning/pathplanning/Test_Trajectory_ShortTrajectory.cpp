/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/SearchPosition.hpp>
#include <srslib_framework/planning/pathplanning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/TrajectoryGenerator.hpp>
#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>
using namespace srs;

typedef SolutionNode<Grid2d> SolutionNodeType;

TEST(Test_Trajectory, ShortTrajectory)
{
    constexpr double DEG90 = AngleMath::deg2rad<double>(90);
    constexpr double DEG180 = AngleMath::deg2rad<double>(180);

    SolutionNodeType SOLUTION_00 = SolutionNodeType(SolutionNodeType::START,
        Pose<>(18, 7, DEG90), Pose<>(18, 7, DEG90));

    SolutionNodeType SOLUTION_01 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(18, 7, DEG90), Pose<>(18, 8, DEG90));

    SolutionNodeType SOLUTION_02 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(18, 8, DEG90), Pose<>(18, 9, DEG90));

    SolutionNodeType SOLUTION_03 = SolutionNodeType(SolutionNodeType::ROTATE,
        Pose<>(18, 9, DEG90), Pose<>(18, 9, DEG180));

    SolutionNodeType SOLUTION_04 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(18, 9, DEG180), Pose<>(17, 9, DEG180));

    SolutionNodeType SOLUTION_05 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(17, 9, DEG180), Pose<>(16, 9, DEG180));

    SolutionNodeType SOLUTION_06 = SolutionNodeType(SolutionNodeType::GOAL,
        Pose<>(16, 9, DEG180), Pose<>(16, 9, DEG180));

    Solution<Grid2d> solution;
    solution.push_back(SOLUTION_00);
    solution.push_back(SOLUTION_01);
    solution.push_back(SOLUTION_02);
    solution.push_back(SOLUTION_03);
    solution.push_back(SOLUTION_04);
    solution.push_back(SOLUTION_05);
    solution.push_back(SOLUTION_06);

    Chuck chuck;
    Trajectory<> trajectory;

    TrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(solution, 0.5);
    solutionConverter.getTrajectory(trajectory);

    cout << solution << endl;
    cout << trajectory << endl;
}
