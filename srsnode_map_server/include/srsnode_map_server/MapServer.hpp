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
#include <srslib_framework/ros/channel/ChannelRosLogicalOccupancyGrid.hpp>
#include <srslib_framework/ros/channel/ChannelRosAmclOccupancyGrid.hpp>
#include <srslib_framework/ros/channel/publisher/PublisherRosCostGrid.hpp>
#include <srslib_framework/ros/function/RosTriggerShutdown.hpp>
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

    void evaluateTriggers();

    void publishMap();

    string frameId_;

    MapStack* mapStack_;
    string mapStackFilename_;

    ChannelMapStack channelMapStack_;
    ChannelRosMapMetadata channelRosMapMetadata_;
    ChannelRosLogicalOccupancyGrid channelRosLogicalOccupancyGrid_;
    ChannelRosAmclOccupancyGrid channelRosAmclOccupancyGrid_;

    RosTriggerShutdown triggerShutdown_;
};

} // namespace srs
