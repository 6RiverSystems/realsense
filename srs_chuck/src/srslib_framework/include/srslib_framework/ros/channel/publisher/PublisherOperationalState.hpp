/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/MsgOperationalState.h>

#include <srslib_framework/ros/channel/publisher/Publisher.hpp>

namespace srs {

class PublisherOperationalState :
    public Publisher<const srslib_framework::MsgOperationalState&, srslib_framework::MsgOperationalState>
{
public:
    PublisherOperationalState(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace)
    {}

    srslib_framework::MsgOperationalState convertData(const srslib_framework::MsgOperationalState& data)
    {
        return data;
    }
};

} // namespace srs
