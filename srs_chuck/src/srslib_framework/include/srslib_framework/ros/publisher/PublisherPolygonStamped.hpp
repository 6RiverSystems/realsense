/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <geometry_msgs/PolygonStamped.h>

#include <srslib_framework/ros/publisher/RosPublisherStamped.hpp>

namespace srs {

class PublisherPolygonStamped :
    public RosPublisherStamped<geometry_msgs::PolygonStamped, const vector<Pose<>>&>
{
public:
    PublisherPolygonStamped(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            RosPublisherStamped(topic, buffer, latched, nameSpace)
    {}

    geometry_msgs::PolygonStamped convertData(const vector<Pose<>>& data, const ros::Time timestamp)
    {
        geometry_msgs::PolygonStamped message;
        message.header.stamp = timestamp;

        vector<geometry_msgs::Point32> polygon;
        for (auto pose : data)
        {
            geometry_msgs::Point32 corner;
            corner.x = pose.x;
            corner.y = pose.y;
            corner.z = 0.0;

            polygon.push_back(corner);
        }
        message.polygon.points = polygon;

        return message;
    }

    void publish(const vector<Pose<>>& data, const string frameId = "map",
        const ros::Time timestamp = ros::Time::now())
    {
        geometry_msgs::PolygonStamped message = convertData(data, timestamp);
        message.header.frame_id = frameId;

        publishMessage(message);
    }
};

} // namespace srs
