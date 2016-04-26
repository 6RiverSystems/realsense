#include <ros/ros.h>

#include <srsnode_position_estimator/PositionEstimator.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS and ROSCPP
    ros::init(argc, argv, "node_pe");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("position_estimator_node started");

    // Create the estimator and run it
    srs::PositionEstimator positionEstimator;
    positionEstimator.run();

    return 0;
}
