/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srsdrv_brainstem/BrainStemMessages.h>
#include <srslib_framework/OdometryRPM.h>
#include <ros/ros.h>

namespace srs {

class RawOdometryPublisher
{
public:

    RawOdometryPublisher(string nameSpace = "~");

    virtual ~RawOdometryPublisher()
    {}

    void publishRawOdometry(srslib_framework::OdometryRPM& rawOdometryMsg);

private:

    ros::NodeHandle rosNodeHandle_;

    ros::Publisher publisher_;
};

} // namespace srs
