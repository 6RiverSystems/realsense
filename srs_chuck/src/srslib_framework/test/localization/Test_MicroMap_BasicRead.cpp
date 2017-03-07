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

TEST(Test_MicroMap, BasicRead)
{
    MapStack* stack = MapStackFactory::fromJsonFile("data/micro-map/micro-map.yaml", 0);

    LogicalMap* logical = stack->getLogicalMap();
    OccupancyMap* occupancy = stack->getOccupancyMap();

    // Check the logical map configuration
    ASSERT_EQ(WeightedGrid2d::PAYLOAD_MAX, logical->getCost(0, 0)) <<
        "The cost is not as expected";
    ASSERT_EQ(CORRECT_COST_111, logical->getCost(1, 0)) <<
        "The cost is not as expected";
    ASSERT_EQ(WeightedGrid2d::PAYLOAD_MIN, logical->getCost(2, 0)) <<
        "The cost is not as expected";
    ASSERT_EQ(CORRECT_COST_111, logical->getCost(0, 1)) <<
        "The cost is not as expected";
    ASSERT_EQ(CORRECT_COST_111, logical->getCost(1, 1)) <<
        "The cost is not as expected";
    ASSERT_EQ(WeightedGrid2d::PAYLOAD_MIN, logical->getCost(2, 1)) <<
        "The cost is not as expected";
    ASSERT_EQ(WeightedGrid2d::PAYLOAD_MIN, logical->getCost(0, 2)) <<
        "The cost is not as expected";
    ASSERT_EQ(WeightedGrid2d::PAYLOAD_MIN, logical->getCost(1, 2)) <<
        "The cost is not as expected";
    ASSERT_EQ(CORRECT_COST_128, logical->getCost(2, 2)) <<
        "The cost is not as expected";

    // Check the logical map configuration
    ASSERT_EQ(SimpleGrid2d::PAYLOAD_MIN, occupancy->getCost(0, 0)) <<
        "The cost is not as expected";
    ASSERT_EQ(SimpleGrid2d::PAYLOAD_MIN, occupancy->getCost(1, 0)) <<
        "The cost is not as expected";
    ASSERT_EQ(SimpleGrid2d::PAYLOAD_MAX, occupancy->getCost(2, 0)) <<
        "The cost is not as expected";
    ASSERT_EQ(SimpleGrid2d::PAYLOAD_MIN, occupancy->getCost(0, 1)) <<
        "The cost is not as expected";
    ASSERT_EQ(SimpleGrid2d::PAYLOAD_MAX, occupancy->getCost(1, 1)) <<
        "The cost is not as expected";
    ASSERT_EQ(SimpleGrid2d::PAYLOAD_MAX, occupancy->getCost(2, 1)) <<
        "The cost is not as expected";
    ASSERT_EQ(SimpleGrid2d::PAYLOAD_MIN, occupancy->getCost(0, 2)) <<
        "The cost is not as expected";
    ASSERT_EQ(SimpleGrid2d::PAYLOAD_MAX, occupancy->getCost(1, 2)) <<
        "The cost is not as expected";
    ASSERT_EQ(SimpleGrid2d::PAYLOAD_NO_INFORMATION, occupancy->getCost(2, 2)) <<
        "The cost is not as expected";

    ASSERT_EQ(WeightedGrid2d::PAYLOAD_MAX, logical->getWeights(2, 0, WeightedGrid2d::ORIENTATION_NORTH)) <<
        "The cost of the weight is not as expected";
    ASSERT_EQ(CORRECT_COST_EAST, logical->getWeights(2, 0, WeightedGrid2d::ORIENTATION_EAST)) <<
        "The cost of the weight is not as expected";
    ASSERT_EQ(CORRECT_COST_SOUTH, logical->getWeights(2, 0, WeightedGrid2d::ORIENTATION_SOUTH)) <<
        "The cost of the weight is not as expected";
    ASSERT_EQ(CORRECT_COST_WEST, logical->getWeights(2, 0, WeightedGrid2d::ORIENTATION_WEST)) <<
        "The cost of the weight is not as expected";
}
