/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/localization/map/logical/LogicalMap.hpp>
#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>

using namespace srs;

TEST(Test_Trajectory, Empty_BasicRead)
{
    LogicalMap* logical = LogicalMapFactory::fromJsonFile("data/empty/empty-logical.geojson");
}
