/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/MapMetadata.h>

#include <srslib_framework/ros/message/MapMessageFactory.hpp>
#include <srslib_framework/ros/publisher/RosPublisherStamped.hpp>

namespace srs {

class PublisherMapMetadata :
    public RosPublisherStamped<srslib_framework::MapMetadata, MapMetadata>
{
public:
    PublisherMapMetadata(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            RosPublisherStamped(topic, buffer, latched, nameSpace)
    {}

    nav_msgs::MapMetaData convertData(MapMetadata data, ros::Time timestamp)
    {
        return MapMessageFactory::mapMetadata2Msg(data, timestamp);
    }
};

} // namespace srs
