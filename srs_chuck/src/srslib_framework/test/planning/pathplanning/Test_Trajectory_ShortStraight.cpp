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
#include <srslib_framework/search/SearchPosition.hpp>
#include <srslib_framework/planning/pathplanning/SolutionNode.hpp>
#include <srslib_framework/planning/pathplanning/Trajectory.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>
using namespace srs;

typedef SolutionNode<Grid2d> SolutionType;

TEST(Test_Trajectory, ShortStraight)
{
    Grid2d grid(10, 10);

    SolutionType SOLUTION_00 = SolutionType(SolutionType::START,
        Pose<>(0, 0, 0));

    SolutionType SOLUTION_01 = SolutionType(SolutionType::FORWARD,
        Pose<>(1, 0, 0));

    SolutionType SOLUTION_02 = SolutionType(SolutionType::FORWARD,
        Pose<>(2, 0, 0));

    SolutionType SOLUTION_03 = SolutionType(SolutionType::GOAL,
        Pose<>(2, 0, 0));

    // Create a sequence of commands
    vector<SolutionNode<Grid2d>> solution = {
        SOLUTION_00,
        SOLUTION_01,
        SOLUTION_02,
        SOLUTION_03
    };

    Chuck chuck;
    Trajectory::TrajectoryType trajectory;

    Trajectory trajectoryConverter(chuck, 0.1);
    trajectoryConverter.calculateTrajectory(solution);
    trajectoryConverter.getTrajectory(trajectory);

    for (auto milestone : trajectory)
    {
        cout << "-------------------------------" << endl;
        cout << milestone.first << milestone.second << endl;
    }
}
