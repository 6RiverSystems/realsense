/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>

#include <srslib_framework/ros/message/PoseMessageFactory.hpp>
#include <srsnode_midbrain/HeadOnCollisionReflex.hpp>

namespace srs {

HeadOnCollisionReflex setUpHOCR()
{
    // Initialize the time so that throttled debug messages do not throw exceptions
    ros::Time::init();
    // Create hocr
    HeadOnCollisionReflex hocr = HeadOnCollisionReflex();

    hocr.setVelocity(Velocity<>::ZERO);
    return hocr;
}

sensor_msgs::LaserScan createLaserScan(float val, float time)
{
    // Create a simple laser scan
    sensor_msgs::LaserScan scan;
    scan.angle_min = -1.5;
    scan.angle_max = 1.5;
    scan.angle_increment = 0.1;
    scan.range_min = 0.1;
    scan.range_max = 20.0;
    // Populate with constant values
    int nscans = (scan.angle_max - scan.angle_min) / scan.angle_increment + 1;
    scan.ranges.resize(nscans, val);

    scan.header.stamp = ros::Time(time);

    return scan;
}

TEST(Test_Reflexes_HeadOnCollisionReflex, Construction)
{
    HeadOnCollisionReflex hocr = setUpHOCR();
    ASSERT_FALSE(hocr.checkHardStop());

    // With one scan, it shouldn't stop
    hocr.setLaserScan(createLaserScan(0.5, 0.0));
    ASSERT_FALSE(hocr.checkHardStop());

    // Even with two scans, it shouldn't stop with no velocity
    hocr.setLaserScan(createLaserScan(0.3, 0.2));
    ASSERT_FALSE(hocr.checkHardStop());
}

TEST(Test_Reflexes_HeadOnCollisionReflex, BasicVelocityTest)
{
    // Create an hocr
    HeadOnCollisionReflex hocr = setUpHOCR();
    hocr.setVelocity(Velocity<>(0.7, 0.0));
    hocr.setLaserScan(createLaserScan(1.0, 0.0));
    ASSERT_FALSE(hocr.checkHardStop());

    hocr.setLaserScan(createLaserScan(0.5, 0.1));
    ASSERT_TRUE(hocr.checkHardStop());
}

}