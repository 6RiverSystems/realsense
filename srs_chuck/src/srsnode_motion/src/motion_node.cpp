#include <ros/ros.h>

#include <srsnode_motion/Motion.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "srsnode_motion");

    ROS_INFO_STREAM("srsnode_motion started");

    // Create the estimator and run it
    srs::Motion motion;
    motion.run();

    return 0;
}
