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
    mapStack_(nullptr)
{
    rosNodeHandle_.param("map_stack", mapStackFilename_,
        string("/home/fsantini/projects/repos/ros/srs_sites/src/srsc_empty/map/empty.yaml"));

    ROS_INFO_STREAM("Target map stack: " << mapStackFilename_);

    rosNodeHandle_.param("/frame_id", frameId_, string("map"));

    mapStack_ = MapStackFactory::fromJsonFile(mapStackFilename_);

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
    channelMapStack_.publish(mapStack_);
    channelRosMapMetadata_.publish(mapStack_->getOccupancyMap()->getMetadata());
    channelRosOccupancyGrid_.publish(mapStack_->getOccupancyMap());
}

} // namespace srs
