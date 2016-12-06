/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/planning/pathplanning/Solution.hpp>
#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/robot_profile/ChuckProfile.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/controller/YoshizawaController.hpp>
#include <srslib_framework/search/SearchPosition.hpp>
using namespace srs;

typedef SolutionNode<Grid2d> SolutionNodeType;

TEST(Test_CMUPathFollower, Usage)
{
    Grid2d grid(10, 10);

    SolutionNodeType SOLUTION_00 = SolutionNodeType(SolutionNodeType::START,
        Pose<>(3, 3, 0));
    SolutionNodeType SOLUTION_01 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(3.1, 3, 0));
    SolutionNodeType SOLUTION_02 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(3.2, 3, 0));
    SolutionNodeType SOLUTION_03 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(3.3, 3, 0));
    SolutionNodeType SOLUTION_04 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(3.4, 3, 0));
    SolutionNodeType SOLUTION_05 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(3.5, 3, 0));
    SolutionNodeType SOLUTION_06 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(3.6, 3, 0));
    SolutionNodeType SOLUTION_07 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(3.7, 3, 0));
    SolutionNodeType SOLUTION_08 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(3.8, 3, 0));
    SolutionNodeType SOLUTION_09 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(3.9, 3, 0));
    SolutionNodeType SOLUTION_10 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(4, 3, 0));
    SolutionNodeType SOLUTION_11 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(4.1, 3, 0));
    SolutionNodeType SOLUTION_12 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(4.2, 3, 0));
    SolutionNodeType SOLUTION_13 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(4.3, 3, 0));
    SolutionNodeType SOLUTION_14 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(4.4, 3, 0));
    SolutionNodeType SOLUTION_15 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(4.5, 3, 0));
    SolutionNodeType SOLUTION_16 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(4.6, 3, 0));
    SolutionNodeType SOLUTION_17 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(4.7, 3, 0));
    SolutionNodeType SOLUTION_18 = SolutionNodeType(SolutionNodeType::FORWARD,
        Pose<>(4.8, 3, 0));
    SolutionNodeType SOLUTION_19 = SolutionNodeType(SolutionNodeType::GOAL,
        Pose<>(4.8, 3, 0));

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
    solution.push_back(SOLUTION_17);
    solution.push_back(SOLUTION_18);
    solution.push_back(SOLUTION_19);

    Chuck chuck;

    Chuck chuck;
    Trajectory<> trajectory;

    TrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromSolution(solution, 0.1);
    solutionConverter.getTrajectory(trajectory);

    ROS_DEBUG_STREAM(solution);
    ROS_DEBUG_STREAM(trajectory);

//    CMUPathFollower controller(1.0, 1.0);

//
//    Trajectory::MilestoneType milestone = *(trajectory.begin() + 1);
//    controller.setReference(milestone.first);
//
//    Pose<> pose(0, 0, 0.1, 0);
//    Velocity<> velocity;

//    Velocity<> command = controller.step(pose);
//    ROS_DEBUG_STREAM(command);
}
