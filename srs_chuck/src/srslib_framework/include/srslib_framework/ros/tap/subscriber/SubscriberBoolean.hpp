/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <srslib_framework/ros/tap/subscriber/RosSubscriberSingleData.hpp>

namespace srs {

class SubscriberBoolean :
    public RosSubscriberSingleData<std_msgs::Bool, bool>
{
public:
    SubscriberBoolean(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            RosSubscriberSingleData(topic, queueLength, nameSpace)
    {}

    ~SubscriberBoolean()
    {}

    bool isNewValueTrue()
    {
        return newDataAvailable() && peek();
    }

    void receiveData(const std_msgs::Bool::ConstPtr message)
    {
        set(message->data);
    }
};

} // namespace srs
