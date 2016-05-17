#include <srssrv_map/MapServer.hpp>

#include <string>
#include <iostream>
#include <fstream>
using namespace std;

#include <ros/ros.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/LinearMath/Quaternion.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MapServer::MapServer() :
    rosNodeHandle_()
{
    string mapFilename;
    rosNodeHandle_.param("target_map", mapFilename, string(""));

    ROS_INFO_STREAM("Target map: " << mapFilename);

    string frame_id;
    rosNodeHandle_.param("frame_id", frame_id, string("map"));

    map_.load(mapFilename);

    pubMapMetadata_ = rosNodeHandle_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    pubMapGrid_ = rosNodeHandle_.advertise<nav_msgs::OccupancyGrid>("map_grid", 1, true);
}

MapServer::~MapServer()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapServer::run()
{
    ros::Rate refreshRate(REFRESH_RATE_HZ);
    while (ros::ok())
    {
        ros::spinOnce();

        publishMap();
        refreshRate.sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

void MapServer::publishMap()
{
    nav_msgs::MapMetaData metadataMessage;

    metadataMessage.map_load_time = ros::Time::now();
    metadataMessage.resolution = map_.getResolution();
    metadataMessage.width = map_.getWidth();
    metadataMessage.height = map_.getHeight();

    geometry_msgs::Pose origin;
    tf::Quaternion orientation = map_.getOrientation();

    origin.position.x = map_.getOrigin().x();
    origin.position.y = map_.getOrigin().y();
    origin.orientation.x = orientation.x();
    origin.orientation.y = orientation.y();
    origin.orientation.z = orientation.z();
    origin.orientation.w = orientation.w();

    metadataMessage.origin = origin;

    pubMapMetadata_.publish(metadataMessage);

    nav_msgs::OccupancyGrid occupancyMessage;

    occupancyMessage.header.stamp = ros::Time::now();
    occupancyMessage.info = metadataMessage;
    map_.getOccupancyGrid(occupancyMessage.data);

    pubMapGrid_.publish(occupancyMessage);
}

} // namespace srs
