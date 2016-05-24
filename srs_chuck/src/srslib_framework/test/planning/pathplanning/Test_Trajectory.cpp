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
typedef SolutionNode<Grid2d>::LocationType LocationType;

//TEST(Test_Trajectory, ShortStraight)
//{
//    Grid2d grid(10, 10);
//
//    SolutionType SOLUTION_00 = SolutionType(
//        SearchActionType(SearchActionType::START,
//            SearchPositionType(LocationType(0, 0), 0)));
//
//    SolutionType SOLUTION_01 = SolutionType(
//        SearchActionType(SearchActionType::FORWARD,
//            SearchPositionType(LocationType(1, 0), 0)));
//
//    SolutionType SOLUTION_02 = SolutionType(
//        SearchActionType(SearchActionType::FORWARD,
//            SearchPositionType(LocationType(2, 0), 0)));
//
//    SolutionType SOLUTION_03 = SolutionType(
//        SearchActionType(SearchActionType::GOAL,
//            SearchPositionType(LocationType(2, 0), 0)));
//
//    // Create a sequence of commands
//    vector<SolutionNode<Grid2d>> solution = {
//        SOLUTION_00,
//        SOLUTION_01,
//        SOLUTION_02,
//        SOLUTION_03
//    };
//
//    Chuck chuck;
//    Trajectory::TrajectoryType trajectory;
//
//    Trajectory trajectoryConverter(chuck, 0.1);
//    trajectoryConverter.getTrajectory(solution, trajectory);
//
//    for (auto milestone : trajectory)
//    {
//        cout << "-------------------------------" << endl;
//        cout << milestone.first << milestone.second << endl;
//    }
//}

//TEST(Test_Trajectory, LongTrajectory)
//{
//    Grid2d grid(10, 10);
//
//    SolutionType SOLUTION_00 = SolutionType(
//        SearchActionType(SearchActionType::START,
//            SearchPositionType(LocationType(0, 0), 90)));
//
//    SolutionType SOLUTION_01 = SolutionType(
//        SearchActionType(SearchActionType::FORWARD,
//            SearchPositionType(LocationType(1, 0), 90)));
//
//    SolutionType SOLUTION_02 = SolutionType(
//        SearchActionType(SearchActionType::FORWARD,
//            SearchPositionType(LocationType(2, 0), 90)));
//
//    SolutionType SOLUTION_03 = SolutionType(
//        SearchActionType(SearchActionType::FORWARD,
//            SearchPositionType(LocationType(3, 0), 90)));
//
//    SolutionType SOLUTION_04 = SolutionType(
//        SearchActionType(SearchActionType::FORWARD,
//            SearchPositionType(LocationType(4, 0), 90)));
//
//    SolutionType SOLUTION_05 = SolutionType(
//        SearchActionType(SearchActionType::FORWARD,
//            SearchPositionType(LocationType(5, 0), 90)));
//
//    SolutionType SOLUTION_06 = SolutionType(
//        SearchActionType(SearchActionType::FORWARD,
//            SearchPositionType(LocationType(6, 0), 90)));
//
//    SolutionType SOLUTION_07 = SolutionType(
//        SearchActionType(SearchActionType::FORWARD,
//            SearchPositionType(LocationType(7, 0), 90)));
//
//    SolutionType SOLUTION_08 = SolutionType(
//        SearchActionType(SearchActionType::FORWARD,
//            SearchPositionType(LocationType(8, 0), 90)));
//
//    SolutionType SOLUTION_09 = SolutionType(
//        SearchActionType(SearchActionType::GOAL,
//            SearchPositionType(LocationType(8, 0), 90)));
//
//    // Create a sequence of commands
//    vector<SolutionNode<Grid2d>> solution = {
//        SOLUTION_00,
//        SOLUTION_01,
//        SOLUTION_02,
//        SOLUTION_03,
//        SOLUTION_04,
//        SOLUTION_05,
//        SOLUTION_06,
//        SOLUTION_07,
//        SOLUTION_08,
//        SOLUTION_09
//    };
//
//    Chuck chuck;
//    Trajectory::TrajectoryType trajectory;
//
//    Trajectory trajectoryConverter(chuck, 0.1);
//    trajectoryConverter.getTrajectory(solution, trajectory);
//
//    for (auto milestone : trajectory)
//    {
//        cout << "-------------------------------" << endl;
//        cout << milestone.first << milestone.second << endl;
//    }
//}

TEST(Test_Trajectory, ShortTrajectory)
{
    Grid2d grid(10, 10);

    SolutionType SOLUTION_00 = SolutionType(SolutionType::START,
        LocationType(0, 0), 0, 0);

    SolutionType SOLUTION_01 = SolutionType(SolutionType::FORWARD,
        LocationType(1, 0), 0, 0);

    SolutionType SOLUTION_02 = SolutionType(SolutionType::FORWARD,
        LocationType(2, 0), 0, 0);

    SolutionType SOLUTION_03 = SolutionType(SolutionType::ROTATE_M90,
        LocationType(2, 0), Math::deg2rad<double>(90), 0);

    SolutionType SOLUTION_04 = SolutionType(SolutionType::FORWARD,
        LocationType(2, 1),  Math::deg2rad<double>(90), 0);

    SolutionType SOLUTION_05 = SolutionType(SolutionType::FORWARD,
        LocationType(2, 2),  Math::deg2rad<double>(90), 0);

    SolutionType SOLUTION_06 = SolutionType(SolutionType::GOAL,
        LocationType(2, 2),  Math::deg2rad<double>(90), 0);

    // Create a sequence of commands
    vector<SolutionNode<Grid2d>> solution = {
        SOLUTION_00,
        SOLUTION_01,
        SOLUTION_02,
        SOLUTION_03,
        SOLUTION_04,
        SOLUTION_05,
        SOLUTION_06
    };

    Chuck chuck;
    Trajectory::TrajectoryType trajectory;

    Trajectory trajectoryConverter(chuck, 1.0 / 20.0);
    trajectoryConverter.calculateTrajectory(solution);
    trajectoryConverter.getTrajectory(trajectory);

    for (auto milestone : trajectory)
    {
        cout << "-------------------------------" << endl;
        cout << milestone.first << milestone.second << endl;
    }
}
