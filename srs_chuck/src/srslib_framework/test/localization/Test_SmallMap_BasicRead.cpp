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

TEST(Test_SmallMap, BasicRead)
{
    MapStack* stack = MapStackFactory::fromJsonFile("data/small-map/small-map.yaml", 0);

    LogicalMap* logical = stack->getLogicalMap();
    OccupancyMap* occupancy = stack->getOccupancyMap();

    cout << *logical << endl;
//
//    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(0, 0)) <<
//        "The cost is not as expected";
//    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(9, 9)) <<
//        "The cost is not as expected";
//    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(9, 30)) <<
//        "The cost is not as expected";
//    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(30, 9)) <<
//        "The cost is not as expected";
//    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(30, 30)) <<
//        "The cost is not as expected";
//
//    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(15, 15)) <<
//        "The cost is not as expected";
//    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(19, 15)) <<
//        "The cost is not as expected";
//    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(15, 19)) <<
//        "The cost is not as expected";
//    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(19, 19)) <<
//        "The cost is not as expected";
//
//    ASSERT_EQ(200, logical->getCost(10, 10)) <<
//        "The cost is not as expected";
//    ASSERT_EQ(200, logical->getCost(29, 10)) <<
//        "The cost is not as expected";
//    ASSERT_EQ(200, logical->getCost(10, 29)) <<
//        "The cost is not as expected";
//    ASSERT_EQ(200, logical->getCost(29, 29)) <<
//        "The cost is not as expected";
//    ASSERT_EQ(200, logical->getCost(22, 22)) <<
//        "The cost is not as expected";
//
//    ASSERT_EQ(0, logical->getCost(23, 23)) <<
//        "The cost is not as expected";
//
//    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getWeights(25, 25, Grid2d::ORIENTATION_NORTH)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(34, logical->getWeights(25, 25, Grid2d::ORIENTATION_EAST)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(23, logical->getWeights(25, 25, Grid2d::ORIENTATION_SOUTH)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(12, logical->getWeights(25, 25, Grid2d::ORIENTATION_WEST)) <<
//        "The cost of the weight is not as expected";
//
//    ASSERT_EQ(11, logical->getWeights(15, 15, Grid2d::ORIENTATION_NORTH)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(11, logical->getWeights(16, 15, Grid2d::ORIENTATION_NORTH)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(11, logical->getWeights(17, 15, Grid2d::ORIENTATION_NORTH)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(11, logical->getWeights(18, 15, Grid2d::ORIENTATION_NORTH)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(11, logical->getWeights(19, 15, Grid2d::ORIENTATION_NORTH)) <<
//        "The cost of the weight is not as expected";
//
//    ASSERT_EQ(22, logical->getWeights(15, 15, Grid2d::ORIENTATION_EAST)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(22, logical->getWeights(16, 15, Grid2d::ORIENTATION_EAST)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(22, logical->getWeights(17, 15, Grid2d::ORIENTATION_EAST)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(22, logical->getWeights(18, 15, Grid2d::ORIENTATION_EAST)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(22, logical->getWeights(19, 15, Grid2d::ORIENTATION_EAST)) <<
//        "The cost of the weight is not as expected";
//
//    ASSERT_EQ(33, logical->getWeights(15, 15, Grid2d::ORIENTATION_SOUTH)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(33, logical->getWeights(16, 15, Grid2d::ORIENTATION_SOUTH)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(33, logical->getWeights(17, 15, Grid2d::ORIENTATION_SOUTH)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(33, logical->getWeights(18, 15, Grid2d::ORIENTATION_SOUTH)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(33, logical->getWeights(19, 15, Grid2d::ORIENTATION_SOUTH)) <<
//        "The cost of the weight is not as expected";
//
//    ASSERT_EQ(Grid2d::WEIGHT_MAX, logical->getWeights(15, 15, Grid2d::ORIENTATION_WEST)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(Grid2d::WEIGHT_MAX, logical->getWeights(16, 15, Grid2d::ORIENTATION_WEST)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(Grid2d::WEIGHT_MAX, logical->getWeights(17, 15, Grid2d::ORIENTATION_WEST)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(Grid2d::WEIGHT_MAX, logical->getWeights(18, 15, Grid2d::ORIENTATION_WEST)) <<
//        "The cost of the weight is not as expected";
//    ASSERT_EQ(Grid2d::WEIGHT_MAX, logical->getWeights(19, 15, Grid2d::ORIENTATION_WEST)) <<
//        "The cost of the weight is not as expected";
}
