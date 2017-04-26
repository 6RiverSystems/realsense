/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/MsgHardwareInfo.h>

#include <srslib_framework/ros/channel/publisher/Publisher.hpp>

namespace srs {

class PublisherHardwareInfo :
    public Publisher<const srslib_framework::MsgHardwareInfo&, srslib_framework::MsgHardwareInfo>
{
public:
    PublisherHardwareInfo(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace)
    {}

    srslib_framework::MsgHardwareInfo convertData(const srslib_framework::MsgHardwareInfo& data)
    {
        return data;
    }
};

} // namespace srs
