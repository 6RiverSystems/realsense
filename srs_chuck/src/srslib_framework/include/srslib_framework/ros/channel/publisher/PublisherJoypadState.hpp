/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/JoypadState.h>

#include <srslib_framework/ros/channel/publisher/Publisher.hpp>
#include <srslib_framework/ros/message/JoypadStateMessageFactory.hpp>

namespace srs {

class PublisherJoypadState :
    public Publisher<const JoypadState&, srslib_framework::JoypadState>
{
public:
    PublisherJoypadState(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace)
    {}

    srslib_framework::JoypadState convertData(const JoypadState& data)
    {
        return JoypadStateMessageFactory::joypadState2Msg(data);
    }
};

} // namespace srs
