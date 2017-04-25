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
#include <srslib_framework/robotics/device/PowerState.hpp>
#include <srslib_framework/ros/message/PowerStateMessageFactory.hpp>

namespace srs {

class PublisherPowerState :
    public Publisher<const PowerState&, srslib_framework::MsgPowerState>
{
public:
	PublisherPowerState(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace)
    {}

	srslib_framework::MsgPowerState convertData(const PowerState& data)
    {
        return PowerStateMessageFactory::powerState2Msg(data);
    }
};

} // namespace srs
