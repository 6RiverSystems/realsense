/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <geometry_msgs/PolygonStamped.h>

#include <srslib_framework/ros/channel/publisher/PublisherStamped.hpp>
#include <srslib_framework/chuck/ChuckTransforms.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/message/PolygonMessageFactory.hpp>

namespace srs {

class PublisherPolygonStamped :
    public PublisherStamped<const vector<Pose<>>&, geometry_msgs::PolygonStamped>
{
public:
    PublisherPolygonStamped(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            PublisherStamped(topic, buffer, latched, nameSpace)
    {}

    geometry_msgs::PolygonStamped convertData(const vector<Pose<>>& data, const ros::Time timestamp)
    {
        return PolygonMessageFactory::poses2PolygonStamped(data, timestamp);
    }

    void publish(const vector<Pose<>>& data,
        const string frameId = ChuckTransforms::MAP,
        const ros::Time timestamp = ros::Time::now())
    {
        geometry_msgs::PolygonStamped message = convertData(data, timestamp);
        message.header.frame_id = frameId;

        publishMessage(message);
    }
};

} // namespace srs
