#include <srsnode_map_server/MapServer.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
using namespace std;

#include <ros/ros.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>

#include <srslib_framework/Map.h>

#include <tf/LinearMath/Quaternion.h>

#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MapServer::MapServer(string name, int argc, char** argv) :
    RosUnit(name, argc, argv, REFRESH_RATE_HZ),
    mapStack_(nullptr),
    pubRosMapMetadata_(ChuckTopics::internal::MAP_ROS_METADATA, 1, true),
    pubMapStack_(ChuckTopics::internal::MAP_STACK, 1, true),
    pubOccupancyGrid_(ChuckTopics::internal::MAP_ROS_OCCUPANCY, 1, true)
{
    string mapFilename;
    rosNodeHandle_.param("map_stack", mapFilename, string("/home/fsantini/projects/repos/ros/srs_sites/src/srsc_empty/map/empty.yaml"));

    ROS_INFO_STREAM("Target map: " << mapFilename);

    string frame_id;
    rosNodeHandle_.param("/frame_id", frame_id, string("map"));

    mapStack_ = MapStackFactory::fromJsonFile(mapFilename);

    serviceMapRequest_ = rosNodeHandle_.advertiseService(ChuckTopics::service::GET_MAP_OCCUPANCY,
        &MapServer::callbackMapRequest, this);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void MapServer::execute()
{
    evaluateTriggers();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapServer::initialize()
{
    publishMap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
bool MapServer::callbackMapRequest(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
    ROS_INFO("Map request service: Sending map");

    vector<int8_t> occupancy;
    OccupancyMapUtils::map2Occupancy(mapStack_->getOccupancyMap(), occupancy);

    res.map.header.stamp = ros::Time::now();
    res.map.info = MapMessageFactory::metadata2RosMsg(mapStack_->getOccupancyMap()->getMetadata());
    res.map.data = occupancy;

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapServer::evaluateTriggers()
{
    if (triggerShutdown_.isTriggerRequested())
    {
        ros::shutdown();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapServer::publishMap()
{
    pubMapStack_.publish(mapStack_);
    pubOccupancyGrid_.publish(mapStack_->getOccupancyMap());
}

} // namespace srs
