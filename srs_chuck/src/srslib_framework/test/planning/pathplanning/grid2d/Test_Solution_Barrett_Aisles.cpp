/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <ros/ros.h>
#include <boost/filesystem.hpp>

#include <srslib_framework/math/AngleMath.hpp>

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>

#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionUtils.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/PoseAdapter.hpp>

#include <srslib_framework/robotics/robot_profile/ChuckProfile.hpp>

#include <srslib_framework/search/AStar.hpp>

#include <srslib_test/planning/pathplanning/PathPlanningUtils.hpp>

using namespace srs;

/*
///////////////////////////////////////////////////////////////////////////////////////////
//                                                                                       //
//                                    Not To Scale                                       //
//                                                                                       //
// ************************************************************************************* //
// ************************************************************************************* //
//                                                                                       //
// I01-L ---------------------------------------------------------------------- I01-R    //
//                                                                                       //
//     H01-T                   G01-T                       F01-T                         //
//      |                        |                           |                           //
//      |     **************     |     *****************     |     *******************   //
//      |     **************     |     *****************     |     *******************   //
//      |     **************     |     *****************     |     *******************   //
//      |     **************     |     *****************     |     *******************   //
//      |     **************     |     *****************     |     *******************   //
//      |     **************     |     *****************     |     *******************   //
//      |     **************     |     *****************     |     *******************   //
//      |     **************     |     *****************     |     *******************   //
//      |     **************     |     *****************     |     *******************   //
//      |     **************     |     *****************     |     *******************   //
//      |     **************     |     *****************     |     *******************   //
//      |     **************     |     *****************     |     *******************   //
//      |     **************     |     *****************     |     *******************   //
//      |     **************     |     *****************     |     *******************   //
//      |                        |                           |                           //
//      |                        |                           |                           //
//     H01-B                   G01-B                       F01-B                         //
//                                                                                       //
//                                                                                       //
//                                                                                       //
//    E01-T                                               CD-T            AB-T           //
//     |     *****************************************     |               |             //
//     |     *****************************************     |     *****     |     *****   //
//     |     *****************************************     |     *****     |     *****   //
//     |     *****************************************     |     *****     |     *****   //
//     |     *****************************************     |     *****     |     *****   //
//     |     *****************************************     |     *****     |     *****   //
//     |     *****************************************     |     *****     |     *****   //
//     |     *****************************************     |     *****     |     *****   //
//     |     *****************************************     |     *****     |     *****   //
//     |     *****************************************     |     *****     |     *****   //
//     |     *****************************************     |     *****     |     *****   //
//     |     *****************************************     |     *****     |     *****   //
//     |     *****************************************     |     *****     |     *****   //
//     |     *****************************************     |     *****     |     *****   //
//     |     *****************************************     |     *****     |     *****   //
//     |     *****************************************     |     *****     |     *****   //
//     |                                                   |               |             //
//     |                                                  CD-B            AB-B           //
//     |                                                                                 //
//     |                                                                                 //
//     |                                                                                 //
//     |             E02-T           E03-T           E04-T                               //
//     |               |               |               |                                 //
//     |     *****     |     *****     |     *****     |     *************************   //
//     |     *****     |     *****     |     *****     |     *************************   //
//     |     *****     |     *****     |     *****     |     *************************   //
//     |     *****     |     *****     |     *****     |     *************************   //
//     |     *****     |     *****     |     *****     |     *************************   //
//     |     *****     |     *****     |     *****     |     *************************   //
//     |     *****     |     *****     |     *****     |     *************************   //
//     |     *****     |     *****     |     *****     |     *************************   //
//     |     *****     |     *****     |     *****     |     *************************   //
//     |     *****     |     *****     |     *****     |     *************************   //
//     |     *****     |     *****     |     *****     |     *************************   //
//     |     *****     |     *****     |     *****     |     *************************   //
//     |     *****     |     *****     |     *****     |     *************************   //
//     |     *****     |     *****     |     *****     |     *************************   //
//     |               |               |               |     *************************   //
//     |               |               |               |     *************************   //
//    E01-B          E02-B           E03-B           E04-B   *************************   //
//                                                           *************************   //
//                                           *****           *************************   //
//                                                           *************************   //
//************************************************************************************   //
//************************************************************************************   //
//                                                                                       //
//                                                                                       //
///////////////////////////////////////////////////////////////////////////////////////////
// y+                                                                                    //
// |                                     90                                              //
// |    Right-Handed Coordinates         |                                               //
// 0--x+                             180-+-0                                             //
//                                       |                                               //
//    E01-B is Robot Start Point        270                                              //
//                                                                                       //
///////////////////////////////////////////////////////////////////////////////////////////

// Aisles are offset the max of 1/2 aisle width from rack OR 1m

//mapNodeI01L: {x: 2030, y: 87835},
//mapNodeI01R: {x: 36360, y: 87835},

const mapNodes = {

    mapNodeF01T: {x: 29905, y: 87835},
    mapNodeF01B: {x: 29905, y: 58020},
    mapNodeG01T: {x: 21910, y: 87835},
    mapNodeG01B: {x: 21910, y: 58020},
    mapNodeH01T: {x: 15290, y: 87835},
    mapNodeH01B: {x: 15290, y: 58020},

    mapNodeCDT: {x: 28890, y: 58020},
    mapNodeCDB: {x: 28890, y: 25450},
    mapNodeABT: {x: 33215, y: 58020},
    mapNodeABB: {x: 33215, y: 25450},

    mapNodeE01T: {x: 14690, y: 58020},
    mapNodeE01B: {x: 14690, y: 4125},
    mapNodeE02T: {x: 19430, y: 25450},
    mapNodeE02B: {x: 19430, y: 4125},
    mapNodeE03T: {x: 23615, y: 25450},
    mapNodeE03B: {x: 23615, y: 4125},
    mapNodeE04T: {x: 27990, y: 25450},
    mapNodeE04B: {x: 27990, y: 4125}
};
*/

static constexpr double DEG0 = 0.0;
static constexpr double DEG270 = AngleMath::deg2Rad<double>(270);

static const Pose<> I01L = Pose<>(2030, 87835, DEG0);
static const Pose<> I01R = Pose<>(36360, 87835, DEG0);

static const Pose<> F01T = Pose<>(29905, 87835, DEG270);
static const Pose<> F01B = Pose<>(29905, 58020, DEG270);
static const Pose<> G01T = Pose<>(21910, 87835, DEG270);
static const Pose<> G01B = Pose<>(21910, 58020, DEG270);
static const Pose<> H01T = Pose<>(15290, 87835, DEG270);
static const Pose<> H01B = Pose<>(15290, 58020, DEG270);

static const Pose<> CDT = Pose<>(28890, 58020, DEG270);
static const Pose<> CDB = Pose<>(28890, 25450, DEG270);
static const Pose<> ABT = Pose<>(33215, 58020, DEG270);
static const Pose<> ABB = Pose<>(33215, 25450, DEG270);

static const Pose<> E01T = Pose<>(14690, 58020, DEG270);
static const Pose<> E01B = Pose<>(14690, 4125, DEG270);
static const Pose<> E02T = Pose<>(19430, 25450, DEG270);
static const Pose<> E02B = Pose<>(19430, 4125, DEG270);
static const Pose<> E03T = Pose<>(23615, 25450, DEG270);
static const Pose<> E03B = Pose<>(23615, 4125, DEG270);
static const Pose<> E04T = Pose<>(27990, 25450, DEG270);
static const Pose<> E04B = Pose<>(27990, 4125, DEG270);

Solution<Grid2dSolutionItem>* calculateSolution(Pose<> start, Pose<> goal)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("pathplanning/grid2d/data/barrett/barrett.yaml");

    Solution<Grid2dSolutionItem>* gridSolution = Grid2dSolutionFactory::fromSingleGoal(
        mapStack->getLogicalMap(), start, goal);

    cout << *gridSolution << endl;

    return gridSolution;
}

// Test of the aisle E01-T ---> E01-B
TEST(Test_Solution, Barrett_Aisles_E01T_E01B)
{
    Solution<Grid2dSolutionItem>* solution = calculateSolution(E01T, E01B);

    ASSERT_FALSE(solution->empty()) << "No solution was found";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";

    double totalCost = Grid2dSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(539, totalCost) << "The cost is not as expected.";
}

// Test of the aisle E02-T ---> E02-B
TEST(Test_Solution, Barrett_Aisles_E02T_E02B)
{
    Solution<Grid2dSolutionItem>* solution = calculateSolution(E02T, E02B);

    ASSERT_FALSE(solution->empty()) << "No solution was found";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";

    double totalCost = Grid2dSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(426, totalCost) << "The cost is not as expected.";
}

// Test of the aisle E03-T ---> E03-B
TEST(Test_Solution, Barrett_Aisles_E03T_E03B)
{
    Solution<Grid2dSolutionItem>* solution = calculateSolution(E03T, E03B);

    ASSERT_FALSE(solution->empty()) << "No solution was found";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";

    double totalCost = Grid2dSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(426, totalCost) << "The cost is not as expected.";
}

// Test of the aisle E04-T ---> E04-B
TEST(Test_Solution, Barrett_Aisles_E04T_E04B)
{
    Solution<Grid2dSolutionItem>* solution = calculateSolution(E04T, E04B);

    ASSERT_FALSE(solution->empty()) << "No solution was found";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";

    double totalCost = Grid2dSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(426, totalCost) << "The cost is not as expected.";
}

// Test of the aisle CD-T ---> CD-B
TEST(Test_Solution, Barrett_Aisles_CDT_CDB)
{
    Solution<Grid2dSolutionItem>* solution = calculateSolution(CDT, CDB);

    ASSERT_FALSE(solution->empty()) << "No solution was found";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";

    double totalCost = Grid2dSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(652, totalCost) << "The cost is not as expected.";
}

// Test of the aisle AB-T ---> AB-B
TEST(Test_Solution, Barrett_Aisles_ABT_ABB)
{
    Solution<Grid2dSolutionItem>* solution = calculateSolution(ABT, ABB);

    ASSERT_FALSE(solution->empty()) << "No solution was found";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";

    double totalCost = Grid2dSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(652, totalCost) << "The cost is not as expected.";
}

// Test of the aisle H01-T ---> H01-B
TEST(Test_Solution, Barrett_Aisles_H01T_H01B)
{
    Solution<Grid2dSolutionItem>* solution = calculateSolution(H01T, H01B);

    ASSERT_FALSE(solution->empty()) << "No solution was found";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";

    double totalCost = Grid2dSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(596, totalCost) << "The cost is not as expected.";
}

// Test of the aisle G01-T ---> G01-B
TEST(Test_Solution, Barrett_Aisles_G01T_G01B)
{
    Solution<Grid2dSolutionItem>* solution = calculateSolution(G01T, G01B);

    ASSERT_FALSE(solution->empty()) << "No solution was found";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";

    double totalCost = Grid2dSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(596, totalCost) << "The cost is not as expected.";
}

// Test of the aisle F01-T ---> F01-B
TEST(Test_Solution, Barrett_Aisles_F01T_F01B)
{
    Solution<Grid2dSolutionItem>* solution = calculateSolution(F01T, F01B);

    ASSERT_FALSE(solution->empty()) << "No solution was found";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";

    double totalCost = Grid2dSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(596, totalCost) << "The cost is not as expected.";
}
