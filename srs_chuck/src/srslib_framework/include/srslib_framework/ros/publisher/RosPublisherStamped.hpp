/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/ros/publisher/RosPublisher.hpp>

namespace srs {

template<typename MESSAGE, typename TYPE>
class RosPublisherStamped :
    public RosPublisher<MESSAGE, TYPE>
{
public:
    RosPublisherStamped(string topic,
        unsigned int buffer,
        bool latched,
        string nameSpace) :
            RosPublisher<MESSAGE, TYPE>(topic, buffer, latched, nameSpace)
    {}

    virtual ~RosPublisherStamped()
    {}

    virtual void publish(TYPE data, const ros::Time timestamp = ros::Time::now())
    {
        MESSAGE message = convertData(data, timestamp);
        RosPublisher<MESSAGE, TYPE>::publishMessage(message);
    }

    MESSAGE convertData(TYPE data)
    {
        return convertData(data, ros::Time::now());
    }

    virtual MESSAGE convertData(TYPE data, const ros::Time timestamp) = 0;
};

} // namespace srs
