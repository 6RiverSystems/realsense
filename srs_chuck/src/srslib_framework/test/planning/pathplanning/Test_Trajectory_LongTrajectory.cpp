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
#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>
using namespace srs;

typedef SolutionNode<Grid2d> SolutionType;

TEST(Test_Trajectory, LongTrajectory)
{
    Grid2d grid(10, 10);

    SolutionType SOLUTION_00 = SolutionType(SolutionType::START,
        Pose<>(0, 0, Math::deg2rad<double>(90)));

    SolutionType SOLUTION_01 = SolutionType(SolutionType::FORWARD,
        Pose<>(1, 0, Math::deg2rad<double>(90)));

    SolutionType SOLUTION_02 = SolutionType(SolutionType::FORWARD,
        Pose<>(2, 0, Math::deg2rad<double>(90)));

    SolutionType SOLUTION_03 = SolutionType(SolutionType::FORWARD,
        Pose<>(3, 0, Math::deg2rad<double>(90)));

    SolutionType SOLUTION_04 = SolutionType(SolutionType::FORWARD,
        Pose<>(4, 0, Math::deg2rad<double>(90)));

    SolutionType SOLUTION_05 = SolutionType(SolutionType::FORWARD,
        Pose<>(5, 0, Math::deg2rad<double>(90)));

    SolutionType SOLUTION_06 = SolutionType(SolutionType::FORWARD,
        Pose<>(6, 0, Math::deg2rad<double>(90)));

    SolutionType SOLUTION_07 = SolutionType(SolutionType::FORWARD,
        Pose<>(7, 0, Math::deg2rad<double>(90)));

    SolutionType SOLUTION_08 = SolutionType(SolutionType::FORWARD,
        Pose<>(8, 0, Math::deg2rad<double>(90)));

    SolutionType SOLUTION_09 = SolutionType(SolutionType::GOAL,
        Pose<>(8, 0, Math::deg2rad<double>(90)));

    // Create a sequence of commands
    vector<SolutionNode<Grid2d>> solution = {
        SOLUTION_00,
        SOLUTION_01,
        SOLUTION_02,
        SOLUTION_03,
        SOLUTION_04,
        SOLUTION_05,
        SOLUTION_06,
        SOLUTION_07,
        SOLUTION_08,
        SOLUTION_09
    };

    Chuck chuck;
//    Trajectory::TrajectoryType trajectory;
//
//    Trajectory trajectoryConverter(chuck, 0.1);
//    trajectoryConverter.calculateTrajectory(solution);
//    trajectoryConverter.getTrajectory(trajectory);
//
//    for (auto milestone : trajectory)
//    {
//        cout << "-------------------------------" << endl;
//        cout << milestone.first << milestone.second << endl;
//    }
}
