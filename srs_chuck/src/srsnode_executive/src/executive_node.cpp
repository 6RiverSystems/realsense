#include <ros/ros.h>

#include <srsnode_executive/Executive.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "srsnode_executive");

    ROS_INFO_STREAM("srsnode_executive started");

    // Create the estimator and run it
    srs::Executive executiveFunctions;
    executiveFunctions.run();

    return 0;
}
