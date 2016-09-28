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

#include <srslib_framework/localization/map/Map.hpp>
#include <srslib_framework/ros/publisher/PublisherRosMapMetadata.hpp>
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

    Map map_;
    vector<int8_t> occupancy_;
    vector<int8_t> notes_;

    PublisherRosMapMetadata pubRosMapMetadata_;
    ros::Publisher pubMapOccupancyGrid_;
    ros::Publisher pubMapCompleteMap_;

    ros::ServiceServer serviceMapRequest_;

    RosTriggerShutdown triggerShutdown_;
};

} // namespace srs
