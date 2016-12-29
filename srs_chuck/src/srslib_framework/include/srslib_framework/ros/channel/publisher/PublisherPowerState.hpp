/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/MsgPowerState.h>

#include <srslib_framework/ros/channel/publisher/Publisher.hpp>

namespace srs {

class PublisherPowerState :
    public Publisher<const srslib_framework::MsgPowerState&, srslib_framework::MsgPowerState>
{
public:
	PublisherPowerState(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace)
    {}

    srslib_framework::MsgPowerState convertData(const srslib_framework::MsgPowerState& data)
    {
        return data;
    }
};

} // namespace srs