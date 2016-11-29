/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <iostream>
#include <srsnode_midbrain/HardStopReflex.hpp>

namespace srs {

HardStopReflex setUpHSR()
{
    // Create an hsr
    HardStopReflex hrs = HardStopReflex();
    // Set a footprint
    std::vector<Pose<>> footprint;
    footprint.push_back(Pose<>(1.0, 1.0, 0.0));
    footprint.push_back(Pose<>(1.0, -1.0, 0.0));
    footprint.push_back(Pose<>(-1.0, -1.0, 0.0));
    footprint.push_back(Pose<>(-1.0, 1.0, 0.0));
    hrs.setFootprint(footprint);

    hrs.setLidarPose(Pose<>(0.5, 0.0, 0.0));
    hrs.setPose(Pose<>::ZERO);
    hrs.setVelocity(Velocity<>::ZERO);

    return hrs;
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
    hsr.setLaserScan(createLaserScan(0.3));
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
    hsr.setLaserScan(createLaserScan(1.3));

    ASSERT_FALSE(hsr.checkHardStop());
}

TEST(Test_Reflexes_HardStopReflex, ForwardVelocity)
{
    HardStopReflex hsr = setUpHSR();
    hsr.setVelocity(Velocity<>(0.1, 0.0));
    hsr.setLaserScan(createLaserScan(1.4));
    ASSERT_FALSE(hsr.checkHardStop());

    hsr.setVelocity(Velocity<>(1.0, 0.0));
    ASSERT_TRUE(hsr.checkHardStop());
    // Printouts
    // std::cout << "Footprint: [" << std::endl;
    // for (auto pt : hsr.getDangerZoneForDisplay())
    // {
    //     std::cout << "  [" << pt.x << "," << pt.y << "]" << std::endl;
    // }
    // std::cout << "]" << std::endl;
    hsr.setLaserScan(createLaserScan(5.0));
    ASSERT_FALSE(hsr.checkHardStop());
}

TEST(Test_Reflexes_HardStopReflex, Arc)
{
    HardStopReflex hsr = setUpHSR();
    hsr.setVelocity(Velocity<>(1.0, 0.5));

    hsr.setLaserScan(createLaserScan(5.0));
    ASSERT_FALSE(hsr.checkHardStop());

    hsr.setLaserScan(createLaserScan(0.75));
    ASSERT_TRUE(hsr.checkHardStop());

}

TEST(Test_Reflexes_HardStopReflex, TurnInPlace)
{
    HardStopReflex hsr = setUpHSR();
    hsr.setVelocity(Velocity<>(0.01, 0.5));

    hsr.setLaserScan(createLaserScan(5.0));
    ASSERT_FALSE(hsr.checkHardStop());

    hsr.setLaserScan(createLaserScan(0.25));
    ASSERT_TRUE(hsr.checkHardStop());

}

TEST(Test_Reflexes_HardStopReflex, DifferentPose)
{
    HardStopReflex hsr = setUpHSR();
    hsr.setVelocity(Velocity<>(0.01, 0.0));
    hsr.setLaserScan(createLaserScan(1.3));
    ASSERT_FALSE(hsr.checkHardStop());

    hsr.setPose(Pose<>(1.0, 0.0, 0.0));
    ASSERT_TRUE(hsr.checkHardStop());
}

TEST(Test_Reflexes_HardStopReflex, WidelyDifferentPose)
{
    HardStopReflex hsr = setUpHSR();
    hsr.setPose(Pose<>(5, 5, -1.5));
    hsr.setVelocity(Velocity<>(0.01, 0.0));
    hsr.setLaserScan(createLaserScan(1.3));
    ASSERT_FALSE(hsr.checkHardStop());

    hsr.setVelocity(Velocity<>(1.0, 0.0));
    ASSERT_TRUE(hsr.checkHardStop());

    hsr.setPose(Pose<>(1.0, 0.0, 0.0));
    ASSERT_FALSE(hsr.checkHardStop());
}

TEST(Test_Reflexes_HardStopReflex, ClearStopCondition)
{
    HardStopReflex hrs = setUpHSR();
}

}