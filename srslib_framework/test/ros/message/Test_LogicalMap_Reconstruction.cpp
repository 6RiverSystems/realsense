/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <ros/ros.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <srslib_framework/LogicalMap.h>
#include <srslib_framework/OccupancyMap.h>

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_framework/localization/map/logical/LogicalMap.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>
#include <srslib_framework/ros/message/OccupancyMapMessageFactory.hpp>
#include <srslib_framework/ros/message/LogicalMapMessageFactory.hpp>
using namespace srs;

TEST(Test_Map, LogicalMapReconstruction)
{
    ros::Time::init();

    MapStack* mapStack = MapStackFactory::fromJsonFile("message/data/small-map/small-map.yaml");
    LogicalMap* correct = mapStack->getLogicalMap();

    srslib_framework::LogicalMap message = LogicalMapMessageFactory::map2Msg(correct);
    LogicalMap* logical = LogicalMapMessageFactory::msg2LogicalMap(message);

    ASSERT_EQ(*logical, *logical) << "The map does not agree with itself";
    ASSERT_EQ(*correct, *logical) << "The marshaled map is not as expected";
}