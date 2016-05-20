#include <ros/ros.h>

#include <srsnode_reflexes/Reflexes.hpp>

int main(int argc, char** argv)
{
    const static string NODE_NAME = "srsnode_reflexes";

    // Initialize ROS
    ros::init(argc, argv, NODE_NAME);
    ROS_INFO_STREAM(NODE_NAME << " started");

    // Create the estimator and run it
    srs::Reflexes reflexes(NODE_NAME);
    reflexes.run();

    return 0;
}
