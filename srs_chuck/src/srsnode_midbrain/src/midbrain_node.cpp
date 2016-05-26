#include <ros/ros.h>

#include <srsnode_midbrain/Midbrain.hpp>

int main(int argc, char** argv)
{
    const static string NODE_NAME = "srsnode_midbrain";

    // Initialize ROS
    ros::init(argc, argv, NODE_NAME);
    ROS_INFO_STREAM(NODE_NAME << " started");

    // Create the estimator and run it
    srs::Midbrain midbrain(NODE_NAME);
    midbrain.run();

    return 0;
}