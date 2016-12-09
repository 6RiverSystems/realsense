/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/MsgSetOperationalState.h>

#include <srslib_framework/ros/channel/publisher/Publisher.hpp>

namespace srs {

class PublisherSetOperationalState :
    public Publisher<const srslib_framework::MsgSetOperationalState&, srslib_framework::MsgSetOperationalState>
{
public:
    PublisherSetOperationalState(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace)
    {}

    srslib_framework::MsgSetOperationalState convertData(const srslib_framework::MsgSetOperationalState& data)
    {
        return data;
    }
};

} // namespace srs
