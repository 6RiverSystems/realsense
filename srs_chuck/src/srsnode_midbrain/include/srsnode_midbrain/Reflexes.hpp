/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <ros/ros.h>

#include <srslib_framework/ros/channel/ChannelDangerZone.hpp>
#include <srslib_framework/ros/channel/ChannelFailedDangerZone.hpp>
#include <srslib_framework/ros/channel/ChannelFailedLaserScan.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemCmd_SetMotionState.hpp>

#include <srslib_framework/ros/tap/TapSensorOdometryPose.hpp>
#include <srslib_framework/ros/tap/RosTapOperationalState.hpp>
#include <srslib_framework/ros/tap/TapFilteredLidar.hpp>
#include <srslib_framework/ros/tap/TapRobotPose.hpp>
#include <srslib_framework/ros/tap/TapLidarPoseOnRobot.hpp>
#include <srslib_framework/ros/tap/TapBrainstem_Connected.hpp>

#include <srslib_framework/platform/timing/MasterTimingDataRecorder.hpp>

#include <srsnode_midbrain/HardStopReflex.hpp>

namespace srs
{

class Reflexes
{
public:
    Reflexes();
    virtual ~Reflexes();

    void execute();

private:
    void readParams();

    // Taps
    TapSensorOdometryPose tapOdometryPose_;
    RosTapOperationalState tapOperationalState_;
    TapFilteredLidar tapFilteredLidar_;

    TapLidarPoseOnRobot tapLidarPoseOnRobot_;
    TapRobotPose tapRobotPose_;
    TapBrainstem_Connected tapBrainstemConnected_;

    // Channels
    ChannelBrainstemCmd_SetMotionState setMotionStateChannel_;
    ChannelDangerZone dangerZoneChannel_;
    ChannelFailedDangerZone fdzChannel_;
    ChannelFailedLaserScan flsChannel_;

    srs::MasterTimingDataRecorder tdr_;

    // Reflexes
    HardStopReflex hardStopReflex_;

    bool brainstemConnected_ = false;

    bool enableHardStopDebugPlotting_ = false;
};

} /* namespace srs */
