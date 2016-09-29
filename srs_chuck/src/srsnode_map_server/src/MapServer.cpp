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

#include <srslib_framework/localization/map/MapFactory.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MapServer::MapServer(string name, int argc, char** argv) :
    RosUnit(name, argc, argv, REFRESH_RATE_HZ),
    occupancy_(),
    pubRosMapMetadata_(ChuckTopics::internal::MAP_ROS_METADATA, 1, true)
{
    string mapFilename;
    rosNodeHandle_.param("target_map", mapFilename, string("/home/fsantini/projects/repos/ros/srs_sites/src/srsc_empty/map/empty.yaml"));

    ROS_INFO_STREAM("Target map: " << mapFilename);

    string frame_id;
    rosNodeHandle_.param("/frame_id", frame_id, string("map"));

    map_.load(mapFilename, ros::Time::now().toSec());
    MapFactory::map2Occupancy(&map_, occupancy_);
    MapFactory::map2Notes(&map_, notes_);

    pubMapOccupancyGrid_ = rosNodeHandle_.advertise<nav_msgs::OccupancyGrid>(
        ChuckTopics::internal::MAP_ROS_OCCUPANCY, 1, true);
    pubMapCompleteMap_ = rosNodeHandle_.advertise<srslib_framework::Map>(
        ChuckTopics::internal::MAP_LOGICAL, 1, true);

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

    res.map.header.stamp = ros::Time::now();

    MapMetadata mapMetadata = map_.getMetadata();
    res.map.info = MapMessageFactory::mapMetadata2RosMsg(mapMetadata);
    res.map.data = occupancy_;

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
    MapMetadata mapMetadata = map_.getMetadata();
    nav_msgs::MapMetaData metadataMessage = MapMessageFactory::mapMetadata2RosMsg(mapMetadata);

    pubRosMapMetadata_.publish(mapMetadata);

    nav_msgs::OccupancyGrid occupancyMessage;

    occupancyMessage.header.stamp = ros::Time::now();
    occupancyMessage.info = metadataMessage;
    occupancyMessage.data = occupancy_;

    pubMapOccupancyGrid_.publish(occupancyMessage);

    srslib_framework::Map mapMessage;

    mapMessage.header.stamp = ros::Time::now();
    mapMessage.info = metadataMessage;
    mapMessage.costs = occupancy_;
    mapMessage.notes = notes_;

    pubMapCompleteMap_.publish(mapMessage);
}

} // namespace srs
