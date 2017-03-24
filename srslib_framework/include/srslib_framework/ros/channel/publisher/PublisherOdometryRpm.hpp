/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/OdometryRpm.h>

#include <srslib_framework/ros/channel/publisher/Publisher.hpp>

namespace srs {

class PublisherOdometryRpm :
    public Publisher<const srslib_framework::OdometryRpm&, srslib_framework::OdometryRpm>
{
public:
	PublisherOdometryRpm(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace)
    {}

    srslib_framework::OdometryRpm convertData(const srslib_framework::OdometryRpm& data)
    {
        return data;
    }
};

} // namespace srs
