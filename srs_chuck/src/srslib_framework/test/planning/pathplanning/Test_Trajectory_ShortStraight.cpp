/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/graph/grid2d/Grid2d.hpp>

#include <srslib_framework/planning/pathplanning/SolutionNode.hpp>
#include <srslib_framework/planning/pathplanning/SimpleSolutionConverter.hpp>

#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>
using namespace srs;

typedef SolutionNode<Grid2d> SolutionNodeType;

TEST(Test_Trajectory, ShortStraight)
{
    SolutionNodeType SOLUTION_00 = SolutionNodeType(SolutionNodeType::START,
        Pose<>(2, 1, 0));

    SolutionNodeType SOLUTION_01 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(2, 2, 0));

    SolutionNodeType SOLUTION_02 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(2, 3, 0));

    SolutionNodeType SOLUTION_03 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(2, 4, 0));

    SolutionNodeType SOLUTION_04 = SolutionNodeType(SolutionNodeType::GOAL,
        Pose<>(2, 4, 0));

    // Create a sequence of commands
    vector<SolutionNodeType> solution = {
        SOLUTION_00,
        SOLUTION_01,
        SOLUTION_02,
        SOLUTION_03,
        SOLUTION_04
    };

    Chuck chuck;
    Trajectory<> trajectory;

    SimpleSolutionConverter solutionConverter(chuck);
    solutionConverter.calculateTrajectory(solution);
    solutionConverter.getTrajectory(trajectory);

    cout << trajectory << endl;
}
