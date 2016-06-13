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

TEST(Test_Trajectory, ShortStraight)
{
    SolutionNodeType SOLUTION_00 = SolutionNodeType(SolutionNodeType::START,
        Pose<>(3, 3, 0), Pose<>(3, 3, 0));

    SolutionNodeType SOLUTION_01 = SolutionNodeType(SolutionNodeType::MOVE,
        Pose<>(3, 3, 0), Pose<>(3, 4, 0));

    SolutionNodeType SOLUTION_02 = SolutionNodeType(SolutionNodeType::GOAL,
        Pose<>(3, 4, 0), Pose<>(3, 4, 0));

    Solution<Grid2d> solution;
    solution.push_back(SOLUTION_00);
    solution.push_back(SOLUTION_01);
    solution.push_back(SOLUTION_02);

    Chuck chuck;
    Trajectory<> trajectory;

    TrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(solution, 0.1);
    solutionConverter.getTrajectory(trajectory);

    cout << solution << endl;
    cout << trajectory << endl;
}
