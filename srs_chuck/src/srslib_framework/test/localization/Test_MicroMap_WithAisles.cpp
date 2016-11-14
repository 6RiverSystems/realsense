/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_framework/localization/map/logical/LogicalMap.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

using namespace srs;

const int CORRECT_COST_128 = 128;
const int CORRECT_COST_111 = 111;
const int CORRECT_COST_EAST = 77;
const int CORRECT_COST_SOUTH = 222;
const int CORRECT_COST_WEST = 111;

TEST(Test_MicroMap, ReadAisles)
{
    MapStack* stack = MapStackFactory::fromJsonFile("data/micro-map-aisles/micro-map-aisles.yaml", 0);

    // The test is not to produce an exception
}
