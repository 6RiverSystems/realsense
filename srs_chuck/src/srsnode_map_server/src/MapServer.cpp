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
#include <srslib_framework/ros/topics/ChuckConfig.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MapServer::MapServer(string name, int argc, char** argv) :
    RosUnit(name, argc, argv, REFRESH_RATE_HZ),
    mapStack_(nullptr)
{
    getLocalParameter(ChuckConfig::Parameters::MAP_STACK, mapStackFilename_, string(""));
    ROS_INFO_STREAM("Target map stack: " << mapStackFilename_);

    getLocalParameter(ChuckConfig::Parameters::FRAME_ID, frameId_, ChuckConfig::Transforms::MAP);
    ROS_INFO_STREAM("Frame id: " << frameId_);

    mapStack_ = MapStackFactory::fromJsonFile(mapStackFilename_);
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
    channelRosLogicalGrid_.publish(mapStack_->getLogicalMap());
    channelRosAmclOccupancyGrid_.publish(mapStack_->getOccupancyMap());

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
