/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/ros/channel/publisher/Publisher.hpp>

namespace srs {

template<typename TYPE, typename MESSAGE>
class PublisherStamped :
    public Publisher<TYPE, MESSAGE>
{
public:
    PublisherStamped(string topic,
        unsigned int buffer,
        bool latched,
        string nameSpace) :
            Publisher<TYPE, MESSAGE>(topic, buffer, latched, nameSpace)
    {}

    virtual ~PublisherStamped()
    {}

    virtual void publish(TYPE data, const ros::Time timestamp = ros::Time::now())
    {
        MESSAGE message = convertData(data, timestamp);
        Publisher<TYPE, MESSAGE>::publishMessage(message);
    }

    MESSAGE convertData(TYPE data)
    {
        return convertData(data, ros::Time::now());
    }

    virtual MESSAGE convertData(TYPE data, const ros::Time timestamp) = 0;
};

} // namespace srs
