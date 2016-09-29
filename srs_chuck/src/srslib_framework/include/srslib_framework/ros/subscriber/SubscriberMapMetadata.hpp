/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <srslib_framework/MapMetadata.h>

#include <srslib_framework/localization/map/MapMetadata.hpp>
#include <srslib_framework/ros/subscriber/RosSubscriberSingleData.hpp>
#include <srslib_framework/ros/message/MapMessageFactory.hpp>

namespace srs {

class SubscriberMapMetadata :
    public RosSubscriberSingleData<srslib_framework::MapMetadata, MapMetadata>
{
public:
    SubscriberMapMetadata(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            RosSubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberMapMetadata()
    {}

    void receiveData(const srslib_framework::MapMetadata::ConstPtr message)
    {
        set(MapMessageFactory::msg2MapMetadata(message));
    }

    void reset()
    {
        RosSubscriber::reset();

        MapMetadata empty;
        set(empty);
    }
};

} // namespace srs
