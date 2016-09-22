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
    public RosPublisherStamped<geometry_msgs::PolygonStamped, vector<Pose<>>>
{
public:
    PublisherPolygonStamped(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            RosPublisherStamped(topic, buffer, latched, nameSpace)
    {}

    geometry_msgs::PolygonStamped convertData(vector<Pose<>> data, ros::Time stamp)
    {
        geometry_msgs::PolygonStamped message;
        message.header.stamp = stamp;

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

    void publish(vector<Pose<>> data, string frameId = "map", ros::Time stamp = ros::Time::now())
    {
        geometry_msgs::PolygonStamped message = convertData(data, stamp);
        message.header.frame_id = frameId;

        publishMessage(message);
    }
};

} // namespace srs
