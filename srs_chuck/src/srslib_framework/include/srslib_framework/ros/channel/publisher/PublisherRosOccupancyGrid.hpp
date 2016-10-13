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
#include <srslib_framework/ros/message/MapMessageFactory.hpp>
#include <srslib_framework/ros/channel/publisher/RosPublisher.hpp>

namespace srs {

class PublisherRosOccupancyGrid :
    public RosPublisher<nav_msgs::OccupancyGrid, const OccupancyMap*>
{
public:
    PublisherRosOccupancyGrid(string topic, string frameId,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            RosPublisher(topic, buffer, latched, nameSpace),
            frameId_(frameId)
    {}

    nav_msgs::OccupancyGrid convertData(const OccupancyMap* data)
    {
        return MapMessageFactory::occupancyMap2Msg(data, frameId_);
    }

private:
    string frameId_;
};

} // namespace srs