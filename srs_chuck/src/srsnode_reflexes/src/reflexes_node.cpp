#include <ros/ros.h>

#include <srsnode_reflexes/Reflexes.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "srsnode_reflexes");

    ROS_INFO_STREAM("srsnode_reflexes started");

    // Create the estimator and run it
    srs::Reflexes reflexes;
    reflexes.run();

    return 0;
}
