/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <srslib_framework/MsgUpdateToteLights.h>

#include <srslib_framework/ros/tap/subscriber/SubscriberSingleData.hpp>

namespace srs {

class SubscriberUpdateToteLights :
    public SubscriberSingleData<srslib_framework::MsgUpdateToteLights, srslib_framework::MsgUpdateToteLights>
{
public:
    SubscriberUpdateToteLights(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            SubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberUpdateToteLights()
    {}

    void receiveData(const srslib_framework::MsgUpdateToteLights::ConstPtr message)
    {
        set(*message);
    }
};

} // namespace srs