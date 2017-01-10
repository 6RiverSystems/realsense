/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <nav_msgs/Odometry.h>

#include <srslib_framework/ros/channel/publisher/Publisher.hpp>

namespace srs {

class PublisherOdometryPose :
    public Publisher<const nav_msgs::Odometry&, nav_msgs::Odometry>
{
public:
	PublisherOdometryPose(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace)
    {}

    nav_msgs::Odometry convertData(const nav_msgs::Odometry& data)
    {
        return data;
    }
};

} // namespace srs
