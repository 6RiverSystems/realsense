/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srsdrv_brainstem/BrainStemMessages.h>
#include <srslib_framework/MsgPowerState.h>
#include <ros/ros.h>

namespace srs {

class PowerStatePublisher
{
public:

    PowerStatePublisher(string nameSpace = "~");

    virtual ~PowerStatePublisher()
    {}

    void publishPowerState(srslib_framework::MsgPowerState& powerStateMsg);

private:

    ros::NodeHandle rosNodeHandle_;

    ros::Publisher publisher_;
};

} // namespace srs
