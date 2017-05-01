/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <srslib_framework/MsgUpdateBodyLights.h>

#include <srslib_framework/ros/tap/subscriber/SubscriberSingleData.hpp>

namespace srs {

class SubscriberUpdateBodyLights :
    public SubscriberSingleData<srslib_framework::MsgUpdateBodyLights, srslib_framework::MsgUpdateBodyLights>
{
public:
    SubscriberUpdateBodyLights(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            SubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberUpdateBodyLights()
    {}

    void receiveData(const srslib_framework::MsgUpdateBodyLights::ConstPtr message)
    {
        set(*message);
    }
};

} // namespace srs
