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

TEST(Test_MicroMap, MapNotes)
{
    MapStack* stack = MapStackFactory::fromJsonFile("data/micro-map-notes/micro-map-notes.yaml", 0);

    LogicalMap* logical = stack->getLogicalMap();
    LogicalMap::LabeledAreaMapType areas = logical->getAreas();

    ASSERT_EQ(2, areas.size()) << "The number of defined areas is not as expected";

    ASSERT_EQ(1, areas.count("test1")) << "The 'test1' area was not defined";
    LogicalMap::LabeledArea area1 = areas["test1"];
    ASSERT_EQ("test1", area1.label) << "The 'test1' area label is not as expected";
    ASSERT_EQ(2, area1.ci) << "The 'test1' area initial X coordinate is not as expected";
    ASSERT_EQ(2, area1.ri) << "The 'test1' area initial Y coordinate is not as expected";
    ASSERT_EQ(2, area1.cf) << "The 'test1' area final X coordinate is not as expected";
    ASSERT_EQ(2, area1.rf) << "The 'test1' area final Y coordinate is not as expected";
    ASSERT_TRUE(area1.notes->has(MapNote::SOUND)) << "The map notes doesn't contain SOUND";
    ASSERT_FALSE(area1.notes->has(MapNote::NONE)) << "The map notes does contain NONE";

    ASSERT_EQ(1, areas.count("test2")) << "The 'test2' area was not defined";
    LogicalMap::LabeledArea area2 = areas["test2"];
    ASSERT_EQ("test2", area2.label) << "The 'test2' area label is not as expected";
    ASSERT_EQ(0, area2.ci) << "The 'test2' area initial X coordinate is not as expected";
    ASSERT_EQ(0, area2.ri) << "The 'test2' area initial Y coordinate is not as expected";
    ASSERT_EQ(2, area2.cf) << "The 'test2' area final X coordinate is not as expected";
    ASSERT_EQ(1, area2.rf) << "The 'test2' area final Y coordinate is not as expected";
    ASSERT_TRUE(area2.notes->has(MapNote::SOUND)) << "The map notes doesn't contain SOUND";
    ASSERT_TRUE(area2.notes->has(MapNote::MAX_VELOCITY)) << "The map notes doesn't contain MAX_VELOCITY";
    ASSERT_FALSE(area2.notes->has(MapNote::NONE)) << "The map notes does contain NONE";
}
