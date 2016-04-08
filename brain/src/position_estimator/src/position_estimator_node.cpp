#include <ros/ros.h>

#include <sensor/SensorFrameQueue.hpp>
#include <sensor/odometry/OdometrySensor.hpp>

#include "PositionEstimator.hpp"

int main(int argc, char** argv)
{
    // Initialize ROS and ROSCPP
    ros::init(argc, argv, "node_pe");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Node Position Estimator started");

    // General queue for the sensor frame
    srs::SensorFrameQueue* sensorFrameQueue = new srs::SensorFrameQueue();

    // Create the estimator and all the input sensors
    srs::PositionEstimator positionEstimator(sensorFrameQueue);

    // Configure the estimator to accept odometry as input sensor
    srs::OdometrySensor* odometrySensor = new srs::OdometrySensor(sensorFrameQueue);
    positionEstimator.addSensor(odometrySensor);

    positionEstimator.run();

    // Dispose of everything allocated
    delete odometrySensor;
    delete sensorFrameQueue;

    return 0;
}
