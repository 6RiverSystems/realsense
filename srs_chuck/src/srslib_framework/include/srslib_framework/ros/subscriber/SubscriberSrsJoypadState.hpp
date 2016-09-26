/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <srslib_framework/JoypadState.h>

#include <srslib_framework/robotics/device/JoypadState.hpp>
#include <srslib_framework/ros/subscriber/RosSubscriberSingleData.hpp>
#include <srslib_framework/ros/message/JoypadStateMessageFactory.hpp>

namespace srs {

class SubscriberSrsJoypadState :
    public RosSubscriberSingleData<srslib_framework::JoypadState, JoypadState>
{
public:
    SubscriberSrsJoypadState(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            RosSubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberSrsJoypadState()
    {}

    void receiveData(const srslib_framework::JoypadState::ConstPtr message)
    {
        set(JoypadStateMessageFactory::msg2JoypadState(message));
    }
};

} // namespace srs
