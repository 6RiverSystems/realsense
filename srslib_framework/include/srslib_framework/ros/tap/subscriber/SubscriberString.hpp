/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <srslib_framework/ros/tap/subscriber/SubscriberSingleData.hpp>

namespace srs {

class SubscriberString :
    public SubscriberSingleData<std_msgs::String, string>
{
public:
    SubscriberString(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            SubscriberSingleData(topic, queueLength, nameSpace)
    {}

    ~SubscriberString()
    {}

    void receiveData(const std_msgs::String::ConstPtr message)
    {
        set(message->data);
    }
};

} // namespace srs
