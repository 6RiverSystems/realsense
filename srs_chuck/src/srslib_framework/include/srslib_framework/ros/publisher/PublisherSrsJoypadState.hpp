/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/JoypadState.h>

#include <srslib_framework/ros/RosPublisher.hpp>
#include <srslib_framework/ros/message/JoypadStateMessageFactory.hpp>

namespace srs {

class PublisherSrsJoypadState :
    public RosPublisher<srslib_framework::JoypadState, JoypadState>
{
public:
    PublisherSrsJoypadState(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            RosPublisher(topic, buffer, latched, nameSpace)
    {}

    srslib_framework::JoypadState convertData(JoypadState data)
    {
        return JoypadStateMessageFactory::joypadState2Msg(data);
    }
};

} // namespace srs
