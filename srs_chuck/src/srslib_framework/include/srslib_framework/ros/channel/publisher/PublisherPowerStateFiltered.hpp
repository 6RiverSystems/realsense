/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/MsgBatteryState.h>

#include <srslib_framework/ros/channel/publisher/Publisher.hpp>

namespace srs {

class PublisherPowerStateFiltered :
    public Publisher<const srslib_framework::MsgBatteryState&, srslib_framework::MsgBatteryState>
{
public:
	PublisherPowerStateFiltered(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace)
    {}

    srslib_framework::MsgBatteryState convertData(const srslib_framework::MsgBatteryState& data)
    {
        return data;
    }
};

} // namespace srs
