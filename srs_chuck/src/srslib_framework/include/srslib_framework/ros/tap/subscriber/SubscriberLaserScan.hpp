/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <sensor_msgs/LaserScan.h>

#include <srslib_framework/ros/tap/subscriber/SubscriberSingleData.hpp>

namespace srs {

class SubscriberLaserScan :
    public SubscriberSingleData<sensor_msgs::LaserScan, sensor_msgs::LaserScan>
{
public:
    SubscriberLaserScan(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            SubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberLaserScan()
    {}

    void receiveData(const sensor_msgs::LaserScan::ConstPtr message)
    {
        set(*message);
    }

    void reset()
    {
        // Reset the data, and then reset the subscriber
        sensor_msgs::LaserScan scan;
        set(scan);

        Subscriber::reset();
    }
};

} // namespace srs
