/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/SensorFrame.h>

#include <srslib_framework/ros/channel/publisher/Publisher.hpp>

namespace srs {

class PublisherSensorFrame :
    public Publisher<const srslib_framework::SensorFrame&, srslib_framework::SensorFrame>
{
public:
	PublisherSensorFrame(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace)
    {}

    srslib_framework::SensorFrame convertData(const srslib_framework::SensorFrame& data)
    {
        return data;
    }
};

} // namespace srs
