#include <ros/ros.h>

#include "PositionEstimator.hpp"

int main(int argc, char** argv)
{
    // Initialize ROS and ROSCPP
    ros::init(argc, argv, "node_pe");
    ros::NodeHandle nh;

    // Create the estimator and run it
    srs::PositionEstimator positionEstimator;
    positionEstimator.run();

    return 0;
}
