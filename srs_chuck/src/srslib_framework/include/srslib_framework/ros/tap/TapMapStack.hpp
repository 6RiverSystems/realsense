/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <srslib_framework/MapStack.h>

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/ros/tap/subscriber/RosSubscriberSingleData.hpp>
#include <srslib_framework/ros/message/MapStackMessageFactory.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class TapMapStack :
    public RosSubscriberSingleData<srslib_framework::MapStack, MapStack*>
{
public:
    TapMapStack() :
            RosSubscriberSingleData(ChuckTopics::internal::MAP_STACK, 1, "~")
    {
        reset();
    }

    ~TapMapStack()
    {}

    void receiveData(const srslib_framework::MapStack::ConstPtr message)
    {
        // Deallocate the current Map Stack
        delete pop();

        // Create a new Map stack from the message
        set(MapStackMessageFactory::msg2MapStack(message));
    }

    void reset()
    {
        RosSubscriber::reset();

        set(nullptr);
    }
};

} // namespace srs
