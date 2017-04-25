/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>

#include <std_msgs/String.h>

#include <srslib_framework/ros/channel/publisher/Publisher.hpp>

namespace srs {

class PublisherString :
    public Publisher<const std::string, std_msgs::String>
{
public:
    PublisherString(std::string topic,
        unsigned int buffer = 100,
        bool latched = false,
        std::string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace)
    {}

    ~PublisherString()
    {}

    std_msgs::String convertData(const std::string data)
    {
        std_msgs::String message;
        message.data = data;

        return message;
    }
};

} // namespace srs
