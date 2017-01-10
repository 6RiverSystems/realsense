/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <srslib_framework/OdometryRpm.h>

#include <srslib_framework/ros/tap/subscriber/SubscriberSingleData.hpp>

namespace srs {

class SubscriberOdometryRpm :
    public SubscriberSingleData<srslib_framework::OdometryRpm, srslib_framework::OdometryRpm>
{
public:
    SubscriberOdometryRpm(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            SubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberOdometryRpm()
    {}

    void receiveData(const srslib_framework::OdometryRpm::ConstPtr message)
    {
        set(*message);
    }
};

} // namespace srs
