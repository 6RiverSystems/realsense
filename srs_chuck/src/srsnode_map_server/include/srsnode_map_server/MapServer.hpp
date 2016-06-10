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

#include <srslib_framework/localization/Map.hpp>
#include <srslib_framework/ros/service/RosTriggerShutdown.hpp>
#include <srslib_framework/MapCoordinates.h>
using namespace srslib_framework;

namespace srs {

class MapServer
{
public:
    MapServer(string nodeName);
    ~MapServer()
    {}

    void run();

private:
    constexpr static double REFRESH_RATE_HZ = 1.0 / 10.0; // [Hz]

    void evaluateTriggers();

    bool onMapCoordinatesRequested(MapCoordinates::Request& req, MapCoordinates::Response& resp);

    void publishMap();

    Map map_;
    vector<int8_t> costsGrid_;
    vector<int8_t> notesGrid_;

    ros::Publisher pubMapMetadata_;
    ros::Publisher pubMapOccupancyGrid_;
    ros::Publisher pubMapCompleteMap_;

    ros::ServiceServer srvMapCoordinates_;

    ros::NodeHandle rosNodeHandle_;

    RosTriggerShutdown triggerShutdown_;
};

} // namespace srs

#endif  // SRS_MAPSERVER_HPP_
