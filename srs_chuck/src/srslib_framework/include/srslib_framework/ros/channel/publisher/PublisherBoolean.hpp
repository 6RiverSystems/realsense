/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <std_msgs/Bool.h>

#include <srslib_framework/ros/channel/publisher/RosPublisher.hpp>

namespace srs {

class PublisherBoolean :
    public RosPublisher<std_msgs::Bool, const bool>
{
public:
    PublisherBoolean(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            RosPublisher(topic, buffer, latched, nameSpace)
    {}

    ~PublisherBoolean()
    {}

    std_msgs::Bool convertData(const bool data)
    {
        std_msgs::Bool message;
        message.data = data;

        return message;
    }
};

} // namespace srs
