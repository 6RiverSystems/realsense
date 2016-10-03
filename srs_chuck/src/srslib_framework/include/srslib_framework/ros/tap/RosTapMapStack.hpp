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

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/ros/subscriber/RosSubscriberSingleData.hpp>
#include <srslib_framework/ros/message/MapMessageFactory.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class RosTapMapStack :
    public RosSubscriberSingleData<srslib_framework::MapStack, MapStack*>
{
public:
    RosTapMapStack() :
            RosSubscriberSingleData(ChuckTopics::internal::MAP_STACK, 1, "~")
    {
        reset();
    }

    ~RosTapMapStack()
    {}

    void receiveData(const srslib_framework::MapStack::ConstPtr message)
    {
        // Deallocate the current Map Stack
        delete pop();

        // Create a new Map stack from the message
        set(MapMessageFactory::msg2MapStack(message));
    }

    void reset()
    {
        RosSubscriber::reset();

        set(nullptr);
    }
};

} // namespace srs
