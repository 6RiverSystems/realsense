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

TEST(Test_Empty, BasicRead)
{
    MapStack* stack = MapStackFactory::fromJsonFile("data/empty/empty.yaml", 0);

    LogicalMap* logical = stack->getLogicalMap();
    OccupancyMap* occupancy = stack->getOccupancyMap();

    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(0, 0)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(19, 14)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(19, 90)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(194, 14)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(194, 90)) <<
        "The cost is not as expected";

    ASSERT_EQ(200, logical->getCost(20, 15)) <<
        "The cost is not as expected";
    ASSERT_EQ(200, logical->getCost(20, 89)) <<
        "The cost is not as expected";
    ASSERT_EQ(200, logical->getCost(193, 15)) <<
        "The cost is not as expected";
    ASSERT_EQ(200, logical->getCost(193, 89)) <<
        "The cost is not as expected";
    ASSERT_EQ(200, logical->getCost(22, 17)) <<
        "The cost is not as expected";
    ASSERT_EQ(200, logical->getCost(22, 87)) <<
        "The cost is not as expected";
    ASSERT_EQ(200, logical->getCost(191, 17)) <<
        "The cost is not as expected";
    ASSERT_EQ(200, logical->getCost(191, 87)) <<
        "The cost is not as expected";

    ASSERT_EQ(Grid2d::PAYLOAD_MIN, logical->getCost(23, 18)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_MIN, logical->getCost(23, 86)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_MIN, logical->getCost(190, 18)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_MIN, logical->getCost(190, 86)) <<
        "The cost is not as expected";
}
