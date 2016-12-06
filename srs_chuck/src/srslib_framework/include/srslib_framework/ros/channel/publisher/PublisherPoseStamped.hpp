/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <geometry_msgs/PoseStamped.h>

#include <srslib_framework/ros/channel/publisher/PublisherStamped.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

class PublisherPoseStamped :
    public PublisherStamped<const Pose<>&, geometry_msgs::PoseStamped>
{
public:
    PublisherPoseStamped(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            PublisherStamped(topic, buffer, latched, nameSpace)
    {}

    geometry_msgs::PoseStamped convertData(const Pose<>& data, const ros::Time timestamp)
    {
        return PoseMessageFactory::pose2PoseStamped(data, timestamp);
    }
};

} // namespace srs
