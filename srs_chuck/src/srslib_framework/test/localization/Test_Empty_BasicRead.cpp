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

    ASSERT_EQ(WeightedGrid2d::PAYLOAD_MAX, logical->getCost(0, 0)) <<
        "The cost is not as expected";
    ASSERT_EQ(WeightedGrid2d::PAYLOAD_MAX, logical->getCost(19, 14)) <<
        "The cost is not as expected";
    ASSERT_EQ(WeightedGrid2d::PAYLOAD_MAX, logical->getCost(19, 90)) <<
        "The cost is not as expected";
    ASSERT_EQ(WeightedGrid2d::PAYLOAD_MAX, logical->getCost(194, 14)) <<
        "The cost is not as expected";
    ASSERT_EQ(WeightedGrid2d::PAYLOAD_MAX, logical->getCost(194, 90)) <<
        "The cost is not as expected";

    ASSERT_EQ(70, logical->getCost(21, 16)) <<
        "The cost is not as expected";
    ASSERT_EQ(70, logical->getCost(21, 89)) <<
        "The cost is not as expected";
    ASSERT_EQ(70, logical->getCost(193, 16)) <<
        "The cost is not as expected";
    ASSERT_EQ(70, logical->getCost(193, 89)) <<
        "The cost is not as expected";
    ASSERT_EQ(70, logical->getCost(23, 18)) <<
        "The cost is not as expected";
    ASSERT_EQ(70, logical->getCost(23, 87)) <<
        "The cost is not as expected";
    ASSERT_EQ(70, logical->getCost(191, 18)) <<
        "The cost is not as expected";
    ASSERT_EQ(70, logical->getCost(191, 87)) <<
        "The cost is not as expected";

    ASSERT_EQ(WeightedGrid2d::PAYLOAD_MIN, logical->getCost(24, 19)) <<
        "The cost is not as expected";
    ASSERT_EQ(WeightedGrid2d::PAYLOAD_MIN, logical->getCost(24, 86)) <<
        "The cost is not as expected";
    ASSERT_EQ(WeightedGrid2d::PAYLOAD_MIN, logical->getCost(190, 19)) <<
        "The cost is not as expected";
    ASSERT_EQ(WeightedGrid2d::PAYLOAD_MIN, logical->getCost(190, 86)) <<
        "The cost is not as expected";
}
