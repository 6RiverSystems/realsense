/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/SearchPosition.hpp>
#include <srslib_framework/search/SolutionNode.hpp>
#include <srslib_framework/planning/pathplanning/Trajectory.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>
#include <srslib_framework/robotics/controller/YoshizawaController.hpp>
using namespace srs;

typedef SolutionNode<Grid2d> SolutionType;
typedef SearchAction<Grid2d> SearchActionType;
typedef SearchPosition<Grid2d> SearchPositionType;
typedef SearchPosition<Grid2d>::LocationType LocationType;

TEST(Test_Yoshizawa, Usage)
{
    Grid2d grid(10, 10);

    SolutionType SOLUTION_00 = SolutionType(
        SearchActionType(SearchActionType::START,
            SearchPositionType(LocationType(0, 0), 90)));

    SolutionType SOLUTION_01 = SolutionType(
        SearchActionType(SearchActionType::FORWARD,
            SearchPositionType(LocationType(1, 0), 90)));

    SolutionType SOLUTION_02 = SolutionType(
        SearchActionType(SearchActionType::GOAL,
            SearchPositionType(LocationType(1, 0), 90)));

    // Create a sequence of commands
    vector<SolutionNode<Grid2d>> solution = {
        SOLUTION_00,
        SOLUTION_01,
        SOLUTION_02
    };

    Chuck chuck;

    Trajectory::TrajectoryType trajectory;

    Trajectory trajectoryConverter(chuck, 0.1);
    trajectoryConverter.getTrajectory(solution, trajectory);

    for (auto milestone : trajectory)
    {
        cout << milestone.first << milestone.second << endl;
    }

    YoshizawaController controller(0.1, 0.9);

    Trajectory::MilestoneType milestone = *(trajectory.begin() + 1);
    controller.setReference(milestone.first);

    Pose<> pose(0, 0, 0.1, 0);
    Velocity<> velocity;

//    Velocity<> command = controller.step(pose);
//    cout << command << endl;
}
