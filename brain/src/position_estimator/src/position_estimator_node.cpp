#include <ros/ros.h>

#include <sensor/odometry/RosOdometer.hpp>

#include "PositionEstimator.hpp"

int main(int argc, char** argv)
{
    // Initialize ROS and ROSCPP
    ros::init(argc, argv, "node_pe");
    ros::NodeHandle nh;

    // Create the estimator and all the input sensors
    srs::PositionEstimator positionEstimator;

    // Configure the estimator to accept odometry as input sensor
    srs::RosOdometer odometer("Odometer");

    positionEstimator.addSensor(&odometer);
    positionEstimator.run();

    return 0;
}
