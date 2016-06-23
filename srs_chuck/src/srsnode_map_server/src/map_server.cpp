#include <ros/ros.h>

#include <srsnode_map_server/MapServer.hpp>

int main(int argc, char** argv)
{
    static const string NODE_NAME = "srsnode_map_server";

    // Initialize ROS
    ros::init(argc, argv, NODE_NAME);
    ROS_INFO_STREAM(NODE_NAME << " started");

    // Create the map server and run it
    srs::MapServer mapServer(NODE_NAME);
    mapServer.run();

    return 0;
}
