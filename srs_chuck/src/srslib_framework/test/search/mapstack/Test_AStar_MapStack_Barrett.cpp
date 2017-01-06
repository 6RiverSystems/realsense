/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/datastructure/Position.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_framework/platform/timing/StopWatch.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackSingleGoal.hpp>
using namespace srs;

#include <valgrind/callgrind.h>

static const int TRIALS = 1;

TEST(Test_AStar_MapStack_Barrett, BigSearch)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("data/barrett/barrett.yaml");

    // Looking for a path between
    // Pose {@: 1.4686e+09, x: 14.0, y: 10.0, t: 1.53943} (140, 100, 90)
    // and
    // Pose {@: 1.4686e+09, x: 73.0, y: 178.0, t: 1.5708} (730, 1780, 90)
    Position startPosition(140, 100, 90);
    Position goalPosition(730, 1780, 90);

    AStar algorithm;
    MapStackNode* start = MapStackNode::instanceOfStart(mapStack, startPosition);
    MapStackSingleGoal* goal = MapStackSingleGoal::instanceOf(goalPosition);

    StopWatch timer;
    //CALLGRIND_START_INSTRUMENTATION;

    //algorithm.search(start, goal);

    //CALLGRIND_STOP_INSTRUMENTATION;
    //CALLGRIND_DUMP_STATS;

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

//    int trials = TRIALS;
//    while (--trials > 0)
//    {
//        algorithm.search(start, goal);
//    }

    float elapsed = timer.elapsedMilliseconds();

    cout << "Elapsed time: " << elapsed << "ms" << endl;
    cout << "Average time: " << elapsed / TRIALS << "ms" << endl;

//    cout << "o: " << algorithm.getOpenNodesCount() << endl;
//    cout << "c: " << algorithm.getClosedNodesCount() << endl;
//    int t = algorithm.getOpenNodesCount() + algorithm.getClosedNodesCount();
//    cout << "t: " << t << endl;
//    cout << "v: " << (1000 * t) / elapsed << "n/s" << endl;
//
//    cout << "replaced_: " << algorithm.replaced_ << endl;
//    cout << "inserted_: " << algorithm.inserted_ << endl;
//    cout << "inClosed_: " << algorithm.inClosed_ << endl;
//    cout << "pruned_: " << algorithm.pruned_ << endl;
}
