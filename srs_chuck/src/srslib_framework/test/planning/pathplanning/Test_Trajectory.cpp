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
using namespace srs;

typedef SolutionNode<Grid2d> SolutionType;
typedef SearchAction<Grid2d> SearchActionType;
typedef SearchPosition<Grid2d> SearchPositionType;
typedef SearchPosition<Grid2d>::LocationType LocationType;

TEST(Test_Trajectory, Solution2Poses)
{
    Grid2d grid(10, 10);

    SolutionType SOLUTION_00 = SolutionType(
        SearchActionType(SearchActionType::START,
            SearchPositionType(LocationType(0, 0), 90)));

    SolutionType SOLUTION_01 = SolutionType(
        SearchActionType(SearchActionType::FORWARD,
            SearchPositionType(LocationType(0, 0), 90)));

    SolutionType SOLUTION_02 = SolutionType(
        SearchActionType(SearchActionType::FORWARD,
            SearchPositionType(LocationType(1, 0), 90)));

    SolutionType SOLUTION_03 = SolutionType(
        SearchActionType(SearchActionType::FORWARD,
            SearchPositionType(LocationType(2, 0), 90)));

    SolutionType SOLUTION_04 = SolutionType(
        SearchActionType(SearchActionType::FORWARD,
            SearchPositionType(LocationType(3, 0), 90)));

    SolutionType SOLUTION_05 = SolutionType(
        SearchActionType(SearchActionType::FORWARD,
            SearchPositionType(LocationType(4, 0), 90)));

    SolutionType SOLUTION_06 = SolutionType(
        SearchActionType(SearchActionType::FORWARD,
            SearchPositionType(LocationType(5, 0), 90)));

    SolutionType SOLUTION_07 = SolutionType(
        SearchActionType(SearchActionType::FORWARD,
            SearchPositionType(LocationType(6, 0), 90)));

    SolutionType SOLUTION_08 = SolutionType(
        SearchActionType(SearchActionType::FORWARD,
            SearchPositionType(LocationType(7, 0), 90)));

    SolutionType SOLUTION_09 = SolutionType(
        SearchActionType(SearchActionType::ROTATE_P90,
            SearchPositionType(LocationType(7, 0), 270)));

    SolutionType SOLUTION_10 = SolutionType(
        SearchActionType(SearchActionType::FORWARD,
            SearchPositionType(LocationType(8, 0), 270)));

    SolutionType SOLUTION_11 = SolutionType(
        SearchActionType(SearchActionType::FORWARD,
            SearchPositionType(LocationType(9, 0), 270)));

    SolutionType SOLUTION_12 = SolutionType(
        SearchActionType(SearchActionType::GOAL,
            SearchPositionType(LocationType(7, 0), 90)));

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
//        SOLUTION_09,
//        SOLUTION_10,
//        SOLUTION_11,
        SOLUTION_12
    };

    Chuck chuck;

    vector<srs::Velocity<>> poses;
    Trajectory trajectoryConverter(solution, chuck, 0.1);
    trajectoryConverter.solution2velocity(Velocity<>(), poses);

    for (auto pose : poses)
    {
        cout << pose << endl;
    }
}
