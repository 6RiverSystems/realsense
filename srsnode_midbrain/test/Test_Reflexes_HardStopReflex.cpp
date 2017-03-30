/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>

#include <srslib_framework/ros/message/PoseMessageFactory.hpp>
#include <srsnode_midbrain/HardStopReflex.hpp>

namespace srs {

HardStopReflex setUpHSR()
{
    // Initialize the time so that throttled debug messages do not throw exceptions
    ros::Time::init();
    // Create an hsr
    HardStopReflex hsr = HardStopReflex();
    // Set a footprint
    std::vector<Pose<>> footprint;
    footprint.push_back(Pose<>(1.0, 1.0, 0.0));
    footprint.push_back(Pose<>(1.0, -1.0, 0.0));
    footprint.push_back(Pose<>(-1.0, -1.0, 0.0));
    footprint.push_back(Pose<>(-1.0, 1.0, 0.0));
    hsr.setFootprint(footprint);

    hsr.setSensorPose(PoseMessageFactory::pose2Transform(Pose<>(0.5, 0.0, 0.0)), LaserScanType::LIDAR);
    hsr.setPose(Pose<>::ZERO);
    hsr.setVelocity(Velocity<>::ZERO);

    // Any violation triggers hard stop (for basic testing)
    hsr.setMaxConsecutiveDangerZoneViolations(0);

    return hsr;
}

sensor_msgs::LaserScan createLaserScan(float val)
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

    return scan;
}

TEST(Test_Reflexes_HardStopReflex, Construction)
{
    HardStopReflex hsr = setUpHSR();
    ASSERT_FALSE(hsr.checkHardStop());
    hsr.setLaserScan(createLaserScan(0.3), LaserScanType::LIDAR);
    ASSERT_FALSE(hsr.checkHardStop());

    // Clear the footprint and check again
    std::vector<Pose<>> footprint;
    hsr.setFootprint(footprint);
    ASSERT_TRUE(hsr.checkHardStop());
}

TEST(Test_Reflexes_HardStopReflex, ZeroVelocity)
{
    // Create an hsr
    HardStopReflex hsr = setUpHSR();
    hsr.setVelocity(Velocity<>(0.0, 0.0));
    hsr.setLaserScan(createLaserScan(1.3), LaserScanType::LIDAR);

    ASSERT_FALSE(hsr.checkHardStop());
}

TEST(Test_Reflexes_HardStopReflex, ForwardVelocity)
{
    HardStopReflex hsr = setUpHSR();
    hsr.setVelocity(Velocity<>(0.1, 0.0));
    hsr.setLaserScan(createLaserScan(1.4), LaserScanType::LIDAR);
    ASSERT_FALSE(hsr.checkHardStop());

    hsr.setVelocity(Velocity<>(1.0, 0.0));
    ASSERT_TRUE(hsr.checkHardStop());

    hsr.setLaserScan(createLaserScan(5.0), LaserScanType::LIDAR);
    ASSERT_FALSE(hsr.checkHardStop());
}

TEST(Test_Reflexes_HardStopReflex, Arc)
{
    HardStopReflex hsr = setUpHSR();
    hsr.setVelocity(Velocity<>(1.0, 0.5));

    hsr.setLaserScan(createLaserScan(5.0), LaserScanType::LIDAR);
    ASSERT_FALSE(hsr.checkHardStop());

    hsr.setLaserScan(createLaserScan(0.75), LaserScanType::LIDAR);
    ASSERT_TRUE(hsr.checkHardStop());

}

TEST(Test_Reflexes_HardStopReflex, TurnInPlace)
{
    HardStopReflex hsr = setUpHSR();
    hsr.setVelocity(Velocity<>(0.01, 0.5));

    hsr.setLaserScan(createLaserScan(5.0), LaserScanType::LIDAR);
    ASSERT_FALSE(hsr.checkHardStop());

    hsr.setLaserScan(createLaserScan(0.25), LaserScanType::LIDAR);
    ASSERT_TRUE(hsr.checkHardStop());

}

TEST(Test_Reflexes_HardStopReflex, DifferentPose)
{
    HardStopReflex hsr = setUpHSR();
    hsr.setVelocity(Velocity<>(0.01, 0.0));
    hsr.setLaserScan(createLaserScan(1.3), LaserScanType::LIDAR);
    ASSERT_FALSE(hsr.checkHardStop());

    hsr.setPose(Pose<>(1.0, 0.0, 0.0));
    ASSERT_TRUE(hsr.checkHardStop());
}

TEST(Test_Reflexes_HardStopReflex, WildlyDifferentPose)
{
    HardStopReflex hsr = setUpHSR();
    hsr.setPose(Pose<>(5, 5, -1.5));
    hsr.setVelocity(Velocity<>(0.01, 0.0));
    hsr.setLaserScan(createLaserScan(1.3), LaserScanType::LIDAR);
    ASSERT_FALSE(hsr.checkHardStop());

    hsr.setVelocity(Velocity<>(1.0, 0.0));
    ASSERT_TRUE(hsr.checkHardStop());

    hsr.setPose(Pose<>(1.0, 0.0, 0.0));
    ASSERT_FALSE(hsr.checkHardStop());
}

TEST(Test_Reflexes_HardStopReflex, MultipleChecksForTrigger)
{
    HardStopReflex hsr = setUpHSR();
    hsr.setVelocity(Velocity<>(1.0, 0.0));
    hsr.setLaserScan(createLaserScan(1.3), LaserScanType::LIDAR);

    hsr.setMaxConsecutiveDangerZoneViolations(2);
    // Hard stop should be true on try > 2;
    ASSERT_FALSE(hsr.checkHardStop());
    ASSERT_FALSE(hsr.checkHardStop());
    ASSERT_TRUE(hsr.checkHardStop());
    ASSERT_TRUE(hsr.checkHardStop());
}

TEST(Test_Reflexes_HardStopReflex, ClearStopCondition)
{
    HardStopReflex hsr = setUpHSR();

    hsr.setVelocity(Velocity<>(1.0, 0.0));
    hsr.setLaserScan(createLaserScan(1.3), LaserScanType::LIDAR);

    // Clearing the hard stop shouldn't happen before a hard stop has been set.
    ASSERT_FALSE(hsr.checkForClear());
    ASSERT_TRUE(hsr.checkHardStop());

    // Clearing the hard stop should not happen while the velocity is high.
    ASSERT_FALSE(hsr.checkForClear());

    // Clearing the hard stop should happen once the velocity is 0.
    hsr.setVelocity(Velocity<>(0.0, 0.0));
    ASSERT_TRUE(hsr.checkForClear());

    // Clear the hard stop should not happen twice in a row.
    ASSERT_FALSE(hsr.checkForClear());
}

}