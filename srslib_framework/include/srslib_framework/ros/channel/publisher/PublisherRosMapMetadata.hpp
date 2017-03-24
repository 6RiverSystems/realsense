/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <nav_msgs/MapMetaData.h>

#include <srslib_framework/localization/map/occupancy/OccupancyMetadata.hpp>
#include <srslib_framework/ros/message/OccupancyMapMessageFactory.hpp>
#include <srslib_framework/ros/channel/publisher/Publisher.hpp>

namespace srs {

class PublisherRosMapMetadata :
    public Publisher<const OccupancyMetadata&, nav_msgs::MapMetaData>
{
public:
    PublisherRosMapMetadata(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace)
    {}

    nav_msgs::MapMetaData convertData(const OccupancyMetadata& data)
    {
        return OccupancyMapMessageFactory::metadata2RosMsg(data);
    }
};

} // namespace srs
