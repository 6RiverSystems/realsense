/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <std_msgs/UInt32.h>

namespace srs {

class PublisherUInt32 :
    public Publisher<const uint32_t, std_msgs::UInt32>
{
public:
	PublisherUInt32(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace)
    {}

    ~PublisherUInt32()
    {}

    std_msgs::Bool convertData(const uint32_t data)
    {
        std_msgs::UInt32 message;
        message.data = data;

        return message;
    }
};

} // namespace srs
