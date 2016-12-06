/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srsdrv_brainstem/BrainStemMessages.h>
#include <srslib_framework/Imu.h>
#include <srslib_framework/Odometry.h>
#include <srslib_framework/SensorFrame.h>
#include <ros/ros.h>

namespace srs {

class SensorFramePublisher
{
public:

    SensorFramePublisher(string nameSpace = "~");

    virtual ~SensorFramePublisher() {}

    void publishImu(srslib_framework::Imu& imuMsg);
    void publishOdometry(srslib_framework::Odometry& odometryMsg);
    void publishSensorFrame(srslib_framework::SensorFrame& sensorFrameMsg);

private:

    ros::NodeHandle rosNodeHandle_;

    ros::Publisher imuPublisher_;
    ros::Publisher odometryPublisher_;
    ros::Publisher sensorFramePublisher_;
};

} // namespace srs
