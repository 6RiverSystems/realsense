#include <ros/ros.h>

#include <srsnode_highlevel_behavior/HighLevelBehavior.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "srsnode_highlevel_behavior");

    ROS_INFO_STREAM("srsnode_highlevel_behavior started");

    // Create the estimator and run it
    srs::HighLevelBehavior highLevelBehavior;
    highLevelBehavior.run();

    return 0;
}
