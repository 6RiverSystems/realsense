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

TEST(Test_AStar_MapStack_Hbc, BigSearch)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("data/Hbc/Hbc.yaml");

    // Looking for a path between
    // {x: 6.570, y: 8.912, t: 0} (131, 178, 0)
    // and
    // {x: 68.255, y: 47.726, t: 0} (1365, 955, 0)
    Position startPosition(131, 178, 0);
    Position goalPosition(1365, 955, 0);

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

    unsigned int counterFoundInClosed_;
    unsigned int counterInserted_;
    unsigned int counterPruned_;
    unsigned int counterReplaced_;

    cout << "replaced: " << algorithm.getReplaced() << " nodes" << endl;
    cout << "inserted: " << algorithm.getInserted() << " nodes" << endl;
    cout << "found in closed: " << algorithm.getFoundInClosed() << " nodes" << endl;
    cout << "pruned: " << algorithm.getPruned() << " nodes" << endl << endl;
}
