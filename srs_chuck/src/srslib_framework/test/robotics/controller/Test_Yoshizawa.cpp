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
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/lowlevel_controller/YoshizawaController.hpp>
using namespace srs;

typedef SolutionNode<Grid2d> SolutionType;

TEST(Test_Yoshizawa, Usage)
{
    Grid2d grid(10, 10);

    SolutionType SOLUTION_00 = SolutionType(SolutionType::START,
        Pose<>(0, 0, Math::deg2rad<double>(90)), 0);

    SolutionType SOLUTION_01 = SolutionType(SolutionType::FORWARD,
        Pose<>(1, 0,  Math::deg2rad<double>(90)), 0);

    SolutionType SOLUTION_02 = SolutionType(SolutionType::GOAL,
        Pose<>(1, 0,  Math::deg2rad<double>(90)), 0);

    // Create a sequence of commands
    vector<SolutionNode<Grid2d>> solution = {
        SOLUTION_00,
        SOLUTION_01,
        SOLUTION_02
    };

    Chuck chuck;

//    Trajectory::TrajectoryType trajectory;
//
//    Trajectory trajectoryConverter(chuck, 1, 0.1);
//    trajectoryConverter.calculateTrajectory(solution);
//    trajectoryConverter.getTrajectory(trajectory);
//
//    for (auto milestone : trajectory)
//    {
//        cout << milestone.first << milestone.second << endl;
//    }
//
//    YoshizawaController controller(0.1, 0.9);
//
//    Trajectory::MilestoneType milestone = *(trajectory.begin() + 1);
//    controller.setReference(milestone.first);
//
//    Pose<> pose(0, 0, 0.1, 0);
//    Velocity<> velocity;

//    Velocity<> command = controller.step(pose);
//    cout << command << endl;
}
