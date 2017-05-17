/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/datastructure/Position.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_timing/StopWatch.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackSingleGoal.hpp>
using namespace srs;

// #define VALGRIND 0
#ifdef VALGRIND
#include <valgrind/callgrind.h>
#endif

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

    #ifdef VALGRIND
        CALLGRIND_START_INSTRUMENTATION;
    #endif

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

    #ifdef VALGRIND
        CALLGRIND_STOP_INSTRUMENTATION;
        CALLGRIND_DUMP_STATS;
    #endif

    float totalElapsed = timer.elapsedMilliseconds();
    cout << "elapsed time: " << totalElapsed << " ms" << endl;

    cout << "open: " << algorithm.getOpenNodesCount() << " nodes" << endl;
    cout << "closed: " << algorithm.getClosedNodesCount() << " nodes" << endl;

    int totalNodes = algorithm.getOpenNodesCount() + algorithm.getClosedNodesCount();
    cout << "total: " << totalNodes << " nodes" << endl;
    cout << "velocity: " << totalNodes / totalElapsed << " kn/s" << endl;

    #if DIAGNOSTICS_ASTAR
        cout << "replaced: " << algorithm.getReplaced() << " nodes" << endl;
        cout << "inserted: " << algorithm.getInserted() << " nodes" << endl;
        cout << "found in closed: " << algorithm.getFoundInClosed() << " nodes" << endl;
        cout << "pruned: " << algorithm.getPruned() << " nodes" << endl << endl;
    #endif
}
