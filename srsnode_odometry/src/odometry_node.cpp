/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <ros/ros.h>
#include <string>

#include <srsnode_odometry/OdometryPositionEstimator.hpp>

int main(int argc, char** argv)
{
    static const std::string NODE_NAME = "srsnode_odometry";

    // Initialize ROS
    ros::init(argc, argv, NODE_NAME);
    ROS_INFO_STREAM(NODE_NAME << " started");

    srs::OdometryPositionEstimator odometryPositionEstimator(NODE_NAME);

    odometryPositionEstimator.run();

    return 0;
}
