/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <srslib_framework/MsgOperationalState.h>

#include <srslib_framework/robotics/device/RobotState.hpp>
#include <srslib_framework/ros/tap/subscriber/SubscriberSingleData.hpp>
#include <srslib_framework/ros/message/RobotStateMessageFactory.hpp>

namespace srs {

class SubscriberRobotState :
    public SubscriberSingleData<srslib_framework::MsgOperationalState, RobotState>
{
public:
    SubscriberRobotState(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            SubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberRobotState()
    {}

    void receiveData(const srslib_framework::MsgOperationalState::ConstPtr message)
    {
        set(RobotStateMessageFactory::msg2RobotState(message));
    }
};

} // namespace srs
