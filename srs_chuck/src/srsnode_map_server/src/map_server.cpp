#include <ros/ros.h>

#include <srssrv_map/MapServer.hpp>

int main(int argc, char** argv)
{
    const static string NODE_NAME = "srsnode_map_server";

    // Initialize ROS
    ros::init(argc, argv, NODE_NAME);
    ROS_INFO_STREAM(NODE_NAME << " started");

    // Create the map server and run it
    srs::MapServer mapServer(NODE_NAME);
    mapServer.run();

    return 0;
}
