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
#include <srslib_framework/localization/map/exception/MapStackException.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>
#include <srslib_framework/chuck/ChuckConfig.hpp>

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

    try {
        mapStack_ = MapStackFactory::fromJsonFile(mapStackFilename_);
    } catch (runtime_error& e) {
        ROS_FATAL("%s", e.what());
        throw;
    }
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
    channelRosLogicalOccupancyGrid_.publish(mapStack_->getLogicalMap());
    channelRosAmclOccupancyGrid_.publish(mapStack_->getOccupancyMap());
}

} // namespace srs
