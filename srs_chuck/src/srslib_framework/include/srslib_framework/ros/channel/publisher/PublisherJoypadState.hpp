/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/JoypadState.h>

#include <srslib_framework/ros/channel/publisher/RosPublisher.hpp>
#include <srslib_framework/ros/message/JoypadStateMessageFactory.hpp>

namespace srs {

class PublisherJoypadState :
    public RosPublisher<srslib_framework::JoypadState, const JoypadState&>
{
public:
    PublisherJoypadState(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            RosPublisher(topic, buffer, latched, nameSpace)
    {}

    srslib_framework::JoypadState convertData(const JoypadState& data)
    {
        return JoypadStateMessageFactory::joypadState2Msg(data);
    }
};

} // namespace srs
