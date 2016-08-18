/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/math/AngleMath.hpp>

#include <srslib_framework/localization/Map.hpp>

#include <srslib_framework/planning/pathplanning/grid/GridSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/grid/PoseAdapter.hpp>

#include <srslib_framework/robotics/robot/Chuck.hpp>

#include <srslib_framework/search/AStar.hpp>

using namespace srs;

/*
//////////////////////////////////////////////////////////////////////
//                                                                  //
//                        Not To Scale                              //
//                                                                  //
//     A               C               E               G            //
//     |               |               |               |            //
//     |     *****     |     *****     |     *****     |     *****  //
//     |     *****     |     *****     |     *****     |     *****  //
//     |     *****     |     *****     |     *****     |     *****  //
//     |     *****     |     *****     |     *****     |     *****  //
//     |     *****     |     *****     |     *****     |     *****  //
//     |     *****     |     *****     |     *****     |     *****  //
//     |     *****     |     *****     |     *****     |     *****  //
//     |     *****     |     *****     |     *****     |     *****  //
//     |     *****     |     *****     |     *****     |     *****  //
//     |     *****     |     *****     |     *****     |     *****  //
//     |     *****     |     *****     |     *****     |     *****  //
//     |     *****     |     *****     |     *****     |     *****  //
//     |     *****     |     *****     |     *****     |     *****  //
//     |     *****     |     *****     |     *****     |     *****  //
//     |               |               |               |            //
//     |               |               |               |            //
//     B               D               F               H            //
//                                                                  //
//                                           *****                  //
//                                                                  //
//     ************************************************************ //
//     ************************************************************ //
//                                                                  //
//                                                                  //
//////////////////////////////////////////////////////////////////////
// y+                                                               //
// |                                     90                         //
// |    Right-Handed Coordinates         |                          //
// 0--x+                             180-+-0                        //
//                                       |                          //
//    B is Robot Start Point            270                         //
//                                                                  //
//////////////////////////////////////////////////////////////////////

// Aisles are offset the max of 1/2 aisle width from rack OR 1m

const mapNodes = {
    mapNodeA: {x: 13113, y: 24759},
    mapNodeB: {x: 13113, y: 4121},
    mapNodeC: {x: 17396, y: 24759},
    mapNodeD: {x: 17396, y: 4121},
    mapNodeE: {x: 21583, y: 24759},
    mapNodeF: {x: 21583, y: 4121},
    mapNodeG: {x: 25959, y: 24759},
    mapNodeH: {x: 25959, y: 4121}
};
*/

void calculateSolution(Map* map, Pose<> fromPose, Pose<> toPose, double correctCost)
{
    // Prepare the start position for the search
    Grid2d::LocationType internalStart;
    int startAngle;
    PoseAdapter::pose2Map(fromPose, map, internalStart, startAngle);

    // Prepare the goal position for the search
    Grid2d::LocationType internalGoal;
    int goalAngle;
    PoseAdapter::pose2Map(toPose, map, internalGoal, goalAngle);

    AStar<Grid2d> algorithm(map->getGrid());
    cout << "Found: " <<
        algorithm.search(SearchPosition<Grid2d>(internalStart, startAngle),
            SearchPosition<Grid2d>(internalGoal, goalAngle)) << endl;

    AStar<Grid2d>::SearchNodeType* solution = algorithm.getSolution();

    ASSERT_TRUE(solution != nullptr) << "No solution exists for search.";

    Solution<GridSolutionItem>* gridSolution = GridSolutionFactory::fromSearch(solution, map);

	double totalCost = GridSolutionFactory::getTotalCost(gridSolution);

	Solution<GridSolutionItem> gridSolution2 = *gridSolution;

	cout << gridSolution2 << endl;
	cout << "Total cost: " << totalCost << endl;

	ASSERT_EQ(correctCost, totalCost) << "The cost is not as expected.";
}

TEST(Test_Trajectory, BarrettMap)
{
    Map* map = new Map();
    map->load("/tmp/srslib_framework/data/barrett.yaml");

    // A ---> B
    Pose<> robotPose = Pose<>(13.113, 24.759, AngleMath::deg2rad<double>(270));
    Pose<> goalPose = Pose<>(13.113, 4.121, AngleMath::deg2rad<double>(270));
    calculateSolution(map, robotPose, goalPose, 414);

    // C ---> D
    robotPose = Pose<>(17.396, 24.759, AngleMath::deg2rad<double>(270));
    goalPose = Pose<>(17.396, 4.121, AngleMath::deg2rad<double>(270));
    calculateSolution(map, robotPose, goalPose, 414);

    // E ---> F
    robotPose = Pose<>(21.583, 24.759, AngleMath::deg2rad<double>(270));
    goalPose = Pose<>(21.583, 4.121, AngleMath::deg2rad<double>(270));
    calculateSolution(map, robotPose, goalPose, 414);

    // G ---> H
    robotPose = Pose<>(25.959, 24.759, AngleMath::deg2rad<double>(270));
    goalPose = Pose<>(25.959, 4.121, AngleMath::deg2rad<double>(270));
    calculateSolution(map, robotPose, goalPose, 414);
}
