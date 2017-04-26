/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <nav_msgs/MapMetaData.h>

#include <srslib_framework/ros/channel/publisher/Publisher.hpp>
#include <srslib_framework/ros/message/LogicalMapMessageFactory.hpp>

namespace srs {

class PublisherRosLogicalOccupancyGrid :
    public Publisher<const LogicalMap*, nav_msgs::OccupancyGrid>
{
public:
    PublisherRosLogicalOccupancyGrid(string topic, string frameId,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace),
            frameId_(frameId)
    {}

    nav_msgs::OccupancyGrid convertData(const LogicalMap* data)
    {
        return LogicalMapMessageFactory::logicalMap2RosMsg(data, frameId_);
    }

private:
    string frameId_;
};

} // namespace srs
