#include <ros/ros.h>

#include <srssrv_map/MapServer.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "srssrv_map");

    ROS_INFO_STREAM("srssrv_map started");

    string mapFilename;

    // Create the map server and run it
    srs::MapServer mapServer;
    mapServer.run();

    return 0;
}
