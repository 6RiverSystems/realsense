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

    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, logical->getCost(0, 0)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, logical->getCost(0, 34)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, logical->getCost(34, 34)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, logical->getCost(34, 0)) <<
        "The cost is not as expected";

    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, logical->getCost(11, 10)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, logical->getCost(11, 22)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, logical->getCost(23, 22)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, logical->getCost(23, 10)) <<
        "The cost is not as expected";

    ASSERT_EQ(70, logical->getCost(12, 11)) <<
        "The cost is not as expected";
    ASSERT_EQ(70, logical->getCost(12, 21)) <<
        "The cost is not as expected";
    ASSERT_EQ(70, logical->getCost(22, 21)) <<
        "The cost is not as expected";
    ASSERT_EQ(70, logical->getCost(22, 11)) <<
        "The cost is not as expected";

    ASSERT_EQ(70, logical->getCost(14, 13)) <<
        "The cost is not as expected";
    ASSERT_EQ(70, logical->getCost(14, 19)) <<
        "The cost is not as expected";
    ASSERT_EQ(70, logical->getCost(20, 19)) <<
        "The cost is not as expected";
    ASSERT_EQ(70, logical->getCost(20, 13)) <<
        "The cost is not as expected";

    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(15, 14)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(15, 18)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(19, 18)) <<
        "The cost is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_MAX, logical->getCost(19, 14)) <<
        "The cost is not as expected";

    LogicalMap::LabeledAreaMapType areas = logical->getAreas();
    ASSERT_EQ(1, areas.size()) << "The number of defined areas is not as expected";

    ASSERT_EQ(1, areas.count("test1")) << "The 'test1' area was not defined";
    LogicalMap::LabeledArea area1 = areas["test1"];
    ASSERT_EQ("test1", area1.label) << "The 'test1' area label is not as expected";
    ASSERT_EQ(11, area1.ci) << "The 'test1' area initial X coordinate is not as expected";
    ASSERT_EQ(24, area1.ri) << "The 'test1' area initial Y coordinate is not as expected";
    ASSERT_EQ(32, area1.cf) << "The 'test1' area final X coordinate is not as expected";
    ASSERT_EQ(31, area1.rf) << "The 'test1' area final Y coordinate is not as expected";
    ASSERT_TRUE(area1.notes->has(MapNote::PLAY_SOUND)) << "The map note does not contain PLAY_SOUND";

}
