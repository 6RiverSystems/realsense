#include <srsnode_map_server/MapServer.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
using namespace std;

#include <ros/ros.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>

#include <srslib_framework/CompleteMap.h>

#include <tf/LinearMath/Quaternion.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MapServer::MapServer(string nodeName) :
    costsGrid_(),
    rosNodeHandle_(nodeName),
    triggerShutdown_(rosNodeHandle_)
{
    string mapFilename;
    rosNodeHandle_.param("/target_map", mapFilename, string(""));
    mapFilename = "/home/fsantini/projects/repos/ros/srs_sites/src/srsc_6rhq/map/6rhq.yaml";

    ROS_INFO_STREAM("Target map: " << mapFilename);

    string frame_id;
    rosNodeHandle_.param("/frame_id", frame_id, string("map"));

    map_.load(mapFilename);
    map_.getCostsGrid(costsGrid_);
    map_.getNotesGrid(notesGrid_);

    pubMapMetadata_ = rosNodeHandle_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    pubMapOccupancyGrid_ = rosNodeHandle_.advertise<nav_msgs::OccupancyGrid>("map_grid", 1, true);
    pubMapCompleteMap_ = rosNodeHandle_.advertise<CompleteMap>("map_complete", 1, true);

    srvMapCoordinates_ = rosNodeHandle_.advertiseService("map_coordinates",
        &MapServer::onMapCoordinatesRequested, this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapServer::run()
{
    triggerShutdown_.connectService();

    // TODO: For not the map is published only once and latched. There should be
    // a more intelligent way to publish the map only when things change
    ros::spinOnce();
    publishMap();

    ros::Rate refreshRate(REFRESH_RATE_HZ);
    while (ros::ok())
    {
        ros::spinOnce();

        evaluateTriggers();

        refreshRate.sleep();
    }
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
bool MapServer::onMapCoordinatesRequested(MapCoordinates::Request& req,
    MapCoordinates::Response& resp)
{
    int r = 0;
    int c = 0;

    map_.getMapCoordinates(req.x, req.y, c, r);

    resp.c = c;
    resp.r = r;

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapServer::publishMap()
{
    nav_msgs::MapMetaData metadataMessage;

    metadataMessage.map_load_time = ros::Time::now();
    metadataMessage.resolution = map_.getResolution();
    metadataMessage.width = map_.getWidthCells();
    metadataMessage.height = map_.getHeightCells();

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
    occupancyMessage.data = costsGrid_;

    pubMapOccupancyGrid_.publish(occupancyMessage);

    CompleteMap mapMessage;

    mapMessage.header.stamp = ros::Time::now();
    mapMessage.info = metadataMessage;
    mapMessage.costs = costsGrid_;
    mapMessage.notes = notesGrid_;

    pubMapCompleteMap_.publish(mapMessage);
}

} // namespace srs
