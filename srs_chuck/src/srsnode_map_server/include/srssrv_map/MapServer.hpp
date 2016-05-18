/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SRS_MAPSERVER_HPP_
#define SRS_MAPSERVER_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/ros/service/RosTriggerShutdown.hpp>

#include <srssrv_map/map/Map.hpp>

namespace srs {

class MapServer
{
public:
    MapServer(string nodeName);
    ~MapServer()
    {}

    void run();

private:
    constexpr static double REFRESH_RATE_HZ = 1;

    void evaluateTriggers();

    void publishMap();

    Map map_;

    ros::Publisher pubMapMetadata_;
    ros::Publisher pubMapGrid_;

    ros::NodeHandle rosNodeHandle_;

    RosTriggerShutdown triggerShutdown_;
};

} // namespace srs

#endif  // SRS_MAPSERVER_HPP_
