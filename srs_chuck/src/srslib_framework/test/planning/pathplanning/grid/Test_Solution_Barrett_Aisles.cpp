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

#include <srslib_framework/localization/Map.hpp>

#include <srslib_framework/planning/pathplanning/grid/GridSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionUtils.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/grid/PoseAdapter.hpp>

#include <srslib_framework/robotics/robot/Chuck.hpp>

#include <srslib_framework/search/AStar.hpp>

#include <srslib_test/localization/MapUtils.hpp>
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

static const Pose<> I01L = Pose<>(2.030, 87.835, DEG0);
static const Pose<> I01R = Pose<>(36.360, 87.835, DEG0);

static const Pose<> F01T = Pose<>(29.905, 87.835, DEG270);
static const Pose<> F01B = Pose<>(29.905, 58.020, DEG270);
static const Pose<> G01T = Pose<>(21.910, 87.835, DEG270);
static const Pose<> G01B = Pose<>(21.910, 58.020, DEG270);
static const Pose<> H01T = Pose<>(15.290, 87.835, DEG270);
static const Pose<> H01B = Pose<>(15.290, 58.020, DEG270);

static const Pose<> CDT = Pose<>(28.890, 58.020, DEG270);
static const Pose<> CDB = Pose<>(28.890, 25.450, DEG270);
static const Pose<> ABT = Pose<>(33.215, 58.020, DEG270);
static const Pose<> ABB = Pose<>(33.215, 25.450, DEG270);

static const Pose<> E01T = Pose<>(14.690, 58.020, DEG270);
static const Pose<> E01B = Pose<>(14.690, 4.125, DEG270);
static const Pose<> E02T = Pose<>(19.430, 25.450, DEG270);
static const Pose<> E02B = Pose<>(19.430, 4.125, DEG270);
static const Pose<> E03T = Pose<>(23.615, 25.450, DEG270);
static const Pose<> E03B = Pose<>(23.615, 4.125, DEG270);
static const Pose<> E04T = Pose<>(27.990, 25.450, DEG270);
static const Pose<> E04B = Pose<>(27.990, 4.125, DEG270);

Solution<GridSolutionItem>* calculateSolution(Pose<> start, Pose<> goal)
{
    Map* map = test::MapUtils::mapFactory("/tmp/srslib_framework/data/barrett.yaml");

    Solution<GridSolutionItem>* solution = GridSolutionFactory::fromGoal(map, start, goal);

    cout << *solution << endl;

    return solution;
}

// Test of the aisle E01-T ---> E01-B
TEST(Test_Solution, Barrett_Aisles_E01T_E01B)
{
    Solution<GridSolutionItem>* solution = calculateSolution(E01T, E01B);

    double totalCost = GridSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(1078, totalCost) << "The cost is not as expected.";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";
}

// Test of the aisle E02-T ---> E02-B
TEST(Test_Solution, Barrett_Aisles_E02T_E02B)
{
    Solution<GridSolutionItem>* solution = calculateSolution(E02T, E02B);

    double totalCost = GridSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(426, totalCost) << "The cost is not as expected.";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";
}

// Test of the aisle E03-T ---> E03-B
TEST(Test_Solution, Barrett_Aisles_E03T_E03B)
{
    Solution<GridSolutionItem>* solution = calculateSolution(E03T, E03B);

    double totalCost = GridSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(426, totalCost) << "The cost is not as expected.";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";
}

// Test of the aisle E04-T ---> E04-B
TEST(Test_Solution, Barrett_Aisles_E04T_E04B)
{
    Solution<GridSolutionItem>* solution = calculateSolution(E04T, E04B);

    double totalCost = GridSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(426, totalCost) << "The cost is not as expected.";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";
}

// Test of the aisle CD-T ---> CD-B
TEST(Test_Solution, Barrett_Aisles_CDT_CDB)
{
    Solution<GridSolutionItem>* solution = calculateSolution(CDT, CDB);

    double totalCost = GridSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(652, totalCost) << "The cost is not as expected.";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";
}

// Test of the aisle AB-T ---> AB-B
TEST(Test_Solution, Barrett_Aisles_ABT_ABB)
{
    Solution<GridSolutionItem>* solution = calculateSolution(ABT, ABB);

    double totalCost = GridSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(652, totalCost) << "The cost is not as expected.";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";
}

// Test of the aisle H01-T ---> H01-B
TEST(Test_Solution, Barrett_Aisles_H01T_H01B)
{
    Solution<GridSolutionItem>* solution = calculateSolution(H01T, H01B);

    double totalCost = GridSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(596, totalCost) << "The cost is not as expected.";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";
}

// Test of the aisle G01-T ---> G01-B
TEST(Test_Solution, Barrett_Aisles_G01T_G01B)
{
    Solution<GridSolutionItem>* solution = calculateSolution(G01T, G01B);

    double totalCost = GridSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(596, totalCost) << "The cost is not as expected.";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";
}

// Test of the aisle F01-T ---> F01-B
TEST(Test_Solution, Barrett_Aisles_F01T_F01B)
{
    Solution<GridSolutionItem>* solution = calculateSolution(F01T, F01B);

    double totalCost = GridSolutionUtils::getTotalCost(*solution);
    ASSERT_EQ(596, totalCost) << "The cost is not as expected.";

    ASSERT_FALSE(test::PathPlanningUtils::checkForNoRotate(solution)) <<
        "Rotate commands were found in the solution";
}
