/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <geometry_msgs/PoseStamped.h>

#include <srslib_framework/ros/channel/publisher/RosPublisherStamped.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

class PublisherPoseStamped :
    public RosPublisherStamped<geometry_msgs::PoseStamped, const Pose<>&>
{
public:
    PublisherPoseStamped(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            RosPublisherStamped(topic, buffer, latched, nameSpace)
    {}

    geometry_msgs::PoseStamped convertData(const Pose<>& data, const ros::Time timestamp)
    {
        return PoseMessageFactory::pose2PoseStamped(data, timestamp);
    }
};

} // namespace srs
