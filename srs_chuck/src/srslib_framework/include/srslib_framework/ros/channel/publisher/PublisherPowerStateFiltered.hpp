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
#include <srslib_framework/ros/message/PowerStateMessageFactory.hpp>

namespace srs {

class PublisherPowerStateFiltered :
    public Publisher<const BatteryState&, srslib_framework::MsgBatteryState>
{
public:
	PublisherPowerStateFiltered(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace)
    {}

	srslib_framework::MsgBatteryState convertData(const BatteryState& data)
    {
        return PowerStateMessageFactory::batteryState2Msg(data);
    }
};

} // namespace srs
