/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/ros/channel/ChannelMapStack.hpp>
#include <srslib_framework/ros/channel/ChannelRosMapMetadata.hpp>
#include <srslib_framework/ros/channel/ChannelRosOccupancyGrid.hpp>
#include <srslib_framework/ros/service/RosTriggerShutdown.hpp>
#include <srslib_framework/ros/unit/RosUnit.hpp>

namespace srs {

class MapServer : public RosUnit<MapServer>
{
public:
    MapServer(string name, int argc, char** argv);
    virtual ~MapServer()
    {}

protected:
    void execute();

    void initialize();

private:
    constexpr static double REFRESH_RATE_HZ = 1.0 / 10.0; // [Hz]

    bool callbackMapRequest(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);

    void evaluateTriggers();

    void publishMap();

    string frameId_;

    MapStack* mapStack_;
    string mapStackFilename_;

    ChannelMapStack channelMapStack_;
    ChannelRosMapMetadata channelRosMapMetadata_;
    ChannelRosOccupancyGrid channelRosOccupancyGrid_;

    ros::ServiceServer serviceMapRequest_;

    RosTriggerShutdown triggerShutdown_;
};

} // namespace srs
