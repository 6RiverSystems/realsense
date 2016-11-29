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
#include <srslib_framework/ros/topics/ChuckTransforms.hpp>
using namespace srs;

TEST(Test_Map, OccupancyMapReconstruction)
{
    ros::Time::init();

    MapStack* mapStack = MapStackFactory::fromJsonFile("message/data/small-map/small-map.yaml");
    OccupancyMap* correct = mapStack->getOccupancyMap();

    srslib_framework::OccupancyMap message = OccupancyMapMessageFactory::map2Msg(correct);
    OccupancyMap* occupancy = OccupancyMapMessageFactory::msg2OccupancyMap(message);

    ASSERT_EQ(*occupancy, *occupancy) << "The map does not agree with itself";
    ASSERT_EQ(*correct, *occupancy) << "The marshaled map is not as expected";
}

TEST(Test_Map, RosOccupancyMapReconstruction)
{
    ros::Time::init();

    MapStack* mapStack = MapStackFactory::fromJsonFile("message/data/small-map/small-map.yaml");
    OccupancyMap* correct = mapStack->getOccupancyMap();

    nav_msgs::OccupancyGrid message = OccupancyMapMessageFactory::occupancyMap2RosMsg(correct,
        ChuckTransforms::MAP);
    OccupancyMap* occupancy = OccupancyMapMessageFactory::msg2OccupancyMap(message);

    ASSERT_EQ(*correct->getGrid(), *occupancy->getGrid()) << "The marshaled map is not as expected";
}
