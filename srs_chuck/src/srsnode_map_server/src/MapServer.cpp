#include <srsnode_map_server/MapServer.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
using namespace std;

#include <ros/ros.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/LinearMath/Quaternion.h>

#include <srslib_framework/localization/map/MapStackFactory.hpp>
#include <srslib_framework/localization/map/MapAdapter.hpp>
#include <srslib_framework/ros/message/OccupancyMapMessageFactory.hpp>
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
        string("/home/fsantini/projects/repos/ros/srs_chuck/src/srsnode_navigation/test/data/one-way/one-way.yaml"));

    ROS_INFO_STREAM("Target map stack: " << mapStackFilename_);

    rosNodeHandle_.param("/frame_id", frameId_, string("map"));

    mapStack_ = MapStackFactory::fromJsonFile(mapStackFilename_);

    serviceMapRequest_ = rosNodeHandle_.advertiseService(ChuckTopics::service::GET_MAP_OCCUPANCY,
        &MapServer::callbackMapRequest, this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
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
    MapAdapter::occupancyMap2Vector(mapStack_->getOccupancyMap(), occupancy);

    res.map.header.stamp = ros::Time::now();
    res.map.info = OccupancyMapMessageFactory::metadata2RosMsg(
        mapStack_->getOccupancyMap()->getMetadata());
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
    // Publish the whole map stack
    channelMapStack_.publish(mapStack_);

    // Publish ROS compatible information
    channelRosMapMetadata_.publish(mapStack_->getOccupancyMap()->getMetadata());
    channelRosOccupancyGrid_.publish(mapStack_->getOccupancyMap());

    channelEastWeightsGrid_.publish(MapAdapter::weights2CostMap2D(
        mapStack_->getLogicalMap(), Grid2d::ORIENTATION_EAST));
    channelNorthWeightsGrid_.publish(MapAdapter::weights2CostMap2D(
        mapStack_->getLogicalMap(), Grid2d::ORIENTATION_NORTH));
    channelSouthWeightsGrid_.publish(MapAdapter::weights2CostMap2D(
        mapStack_->getLogicalMap(), Grid2d::ORIENTATION_SOUTH));
    channelWestWeightsGrid_.publish(MapAdapter::weights2CostMap2D(
        mapStack_->getLogicalMap(), Grid2d::ORIENTATION_WEST));
}

} // namespace srs
