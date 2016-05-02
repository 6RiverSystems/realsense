#include <ros/ros.h>

#include <srsnode_position_estimator/PositionEstimator.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "srsnode_position_estimator");

    ROS_INFO_STREAM("srsnode_position_estimator started");

    // Create the estimator and run it
    srs::PositionEstimator positionEstimator;
    positionEstimator.run();

    return 0;
}
