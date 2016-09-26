/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
using namespace std;

#include <srslib_framework/robotics/robot_profile/ChuckProfile.hpp>
using namespace srs;

TEST(Test_Chuck, Usage)
{
    Chuck robotProfile;

    ROS_DEBUG_STREAM(robotProfile.bodyWidth);
}
