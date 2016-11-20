/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <costmap_2d/costmap_2d_ros.h>

#include <nav_msgs/MapMetaData.h>

#include <srslib_framework/ros/message/CostMap2DMessageFactory.hpp>
#include <srslib_framework/ros/channel/publisher/PublisherStamped.hpp>

namespace srs {

class PublisherRosCostGrid :
    public PublisherStamped<nav_msgs::OccupancyGrid, const costmap_2d::Costmap2D*>
{
public:
    PublisherRosCostGrid(string topic, string frameId,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            PublisherStamped(topic, buffer, latched, nameSpace),
            frameId_(frameId)
    {}

    nav_msgs::OccupancyGrid convertData(const costmap_2d::Costmap2D* data,
        const ros::Time timestamp)
    {
        return CostMap2DMessageFactory::costMap2RosMsg(data, frameId_, timestamp);
    }

private:
    string frameId_;
};

} // namespace srs
