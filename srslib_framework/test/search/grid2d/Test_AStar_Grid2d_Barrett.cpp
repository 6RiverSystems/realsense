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
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_timing/StopWatch.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dNode.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dSingleGoal.hpp>
using namespace srs;

static const int TRIALS = 10;

TEST(Test_AStar_Grid2d_Barrett, BigSearch)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile(
        "data/barrett/barrett.yaml");

    // Looking for a path between
    // Pose {@: 1.4686e+09, x: 14.0, y: 10.0, t: 1.53943} (140, 100, 90)
    // and
    // Pose {@: 1.4686e+09, x: 73.0, y: 178.0, t: 1.5708} (730, 1780, 90)
    Grid2d::Position startPosition(140, 100, 90);
    Grid2d::Position goalPosition(730, 1780, 90);

    AStar algorithm;
    Grid2dNode* start = Grid2dNode::instanceOfStart(mapStack->getLogicalMap()->getGrid(),
        startPosition);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(goalPosition);

    ASSERT_TRUE(algorithm.search(start, goal)) <<
        "A plan was not found";

    StopWatch timer;

    int trials = TRIALS;
    while (--trials > 0)
    {
        algorithm.search(start, goal);
    }

    float elapsed = timer.elapsedMilliseconds();

    cout << "Elapsed time: " << elapsed << "ms" << endl;
    cout << "Average time: " << elapsed / TRIALS << "ms" << endl;
}
