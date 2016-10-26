/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/MapStack.h>

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/ros/message/MapStackMessageFactory.hpp>
#include <srslib_framework/ros/channel/publisher/RosPublisher.hpp>

namespace srs {

class PublisherMapStack :
    public RosPublisher<srslib_framework::MapStack, const MapStack*>
{
public:
    PublisherMapStack(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            RosPublisher(topic, buffer, latched, nameSpace)
    {}

    srslib_framework::MapStack convertData(const MapStack* data)
    {
        return MapStackMessageFactory::mapStack2Msg(data);
    }
};

} // namespace srs
