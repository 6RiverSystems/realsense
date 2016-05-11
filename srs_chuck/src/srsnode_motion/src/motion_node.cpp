#include <ros/ros.h>

#include <srsnode_motion/Motion.hpp>

int main(int argc, char** argv)
{
    const static string NODE_NAME = "srsnode_motion";

    // Initialize ROS
    ros::init(argc, argv, NODE_NAME);
    ROS_INFO_STREAM(NODE_NAME << " started");

    // Create the estimator and run it
    srs::Motion motion(NODE_NAME);
    motion.run();

    return 0;
}
