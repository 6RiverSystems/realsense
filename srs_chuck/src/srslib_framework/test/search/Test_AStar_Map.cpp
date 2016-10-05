/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_framework/search/AStar.hpp>

#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;

TEST(Test_AStar, WithMap)
{
    Grid2d::LocationType start(86, 48);
    Grid2d::LocationType goal(87, 48);

    MapStack* mapStack = MapStackFactory::fromJsonFile("data/6rshq/6rshq.yaml");

    test::MemoryWatch memoryWatch;

    AStar<Grid2d>* algorithm = new AStar<Grid2d>(mapStack->getOccupancyMap()->getGrid());

    ROS_DEBUG_STREAM("Found: " <<
        algorithm->search(SearchPosition<Grid2d>(start, 0), SearchPosition<Grid2d>(goal, 0)));

    algorithm->clear();

    delete algorithm;

    ROS_DEBUG_STREAM("Memory usage: " << memoryWatch.getMemoryUsage());
    ROS_DEBUG_STREAM("Memory leaks: " << !memoryWatch.isZero());
}
