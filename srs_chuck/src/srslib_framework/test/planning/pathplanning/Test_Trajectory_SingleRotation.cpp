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

TEST(Test_Trajectory, SingleRotation)
{
    constexpr double DEG90 = AngleMath::deg2rad<double>(90);
    constexpr double DEG180 = AngleMath::deg2rad<double>(180);

    SolutionNodeType SOLUTION_00 = SolutionNodeType(SolutionNodeType::ROTATE,
        Pose<>(18, 9, DEG90), Pose<>(18, 9, DEG180));

    Solution<Grid2d> solution;
    solution.push_back(SOLUTION_00);
    cout << solution << endl;

    Chuck chuck;
    Trajectory<> trajectory;

    TrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(solution);
    solutionConverter.getTrajectory(trajectory);

    cout << trajectory << endl;
}
