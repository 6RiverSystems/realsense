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

using namespace srs;

TEST(Test_Empty, BasicRead)
{
    MapStack* stack = MapStackFactory::fromJsonFile("data/empty/empty.yaml", 0);

    LogicalMap* logical = stack->getLogicalMap();
    OccupancyMap* occupancy = stack->getOccupancyMap();

    cout << *logical << endl;

    ASSERT_EQ(Grid2d::COST_MAX, logical->getCost(0, 0)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::COST_MAX, logical->getCost(17, 12)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::COST_MAX, logical->getCost(17, 93)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::COST_MAX, logical->getCost(195, 92)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::COST_MAX, logical->getCost(195, 93)) <<
        "The cost is not as expected";
}
