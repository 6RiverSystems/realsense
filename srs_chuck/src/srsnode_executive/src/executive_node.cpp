#include <ros/ros.h>

#include <srsnode_executive/Executive.hpp>

int main(int argc, char** argv)
{
    const static string NODE_NAME = "srsnode_executive";

    // Initialize ROS
    ros::init(argc, argv, NODE_NAME);
    ROS_INFO_STREAM(NODE_NAME << " started");

    // Create the estimator and run it
    srs::Executive executiveFunctions(NODE_NAME);
    executiveFunctions.run();

    return 0;
}
