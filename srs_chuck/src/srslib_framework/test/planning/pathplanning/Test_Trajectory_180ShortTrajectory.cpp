/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/math/Math.hpp>
#include <srslib_framework/graph/grid2d/Grid2d.hpp>
//#include <srslib_framework/search/SearchPosition.hpp>
#include <srslib_framework/planning/pathplanning/Solution.hpp>
#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>
#include <srslib_framework/planning/pathplanning/SimpleSolutionConverter.hpp>
using namespace srs;

typedef SolutionNode<Grid2d> SolutionType;

TEST(Test_Trajectory, 180ShortTrajectory)
{
    Grid2d grid(10, 10);

    SolutionType SOLUTION_00 = SolutionType(SolutionType::START,
        Pose<>(18, 7, Math::deg2rad<double>(90)));

    SolutionType SOLUTION_01 = SolutionType(SolutionType::FORWARD,
        Pose<>(18, 7.1, Math::deg2rad<double>(90)));

    SolutionType SOLUTION_02 = SolutionType(SolutionType::FORWARD,
        Pose<>(18, 7.2, Math::deg2rad<double>(90)));

    SolutionType SOLUTION_03 = SolutionType(SolutionType::ROTATE_P90,
        Pose<>(18, 7.2, Math::deg2rad<double>(180)));

    SolutionType SOLUTION_04 = SolutionType(SolutionType::FORWARD,
        Pose<>(17.9, 7.2,  Math::deg2rad<double>(180)));

    SolutionType SOLUTION_05 = SolutionType(SolutionType::FORWARD,
        Pose<>(17.8, 7.2,  Math::deg2rad<double>(180)));

    SolutionType SOLUTION_06 = SolutionType(SolutionType::GOAL,
        Pose<>(17.8, 7.2,  Math::deg2rad<double>(180)));

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

    SimpleSolutionConverter solutionConverter(chuck);
    solutionConverter.calculateTrajectory(solution);
    solutionConverter.getTrajectory(trajectory);

    cout << solution << endl;
    cout << trajectory << endl;
}
