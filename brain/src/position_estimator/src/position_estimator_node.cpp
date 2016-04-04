#include <ros/ros.h>

#include "framework/SensorFrameQueue.hpp"
#include "PositionEstimator.hpp"

#include "sensor/OdometrySensor.hpp"

int main(int argc, char** argv)
{
    // Initialize ROS and ROSCPP
    ros::init(argc, argv, "node_pe");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Node Position Estimator started");

    // General queue for the sensor frame
    SensorFrameQueue* sensorFrameQueue = new SensorFrameQueue();

    // Create the estimator and all the input sensors
    PositionEstimator positionEstimator(sensorFrameQueue);

    // Configure the estimator to accept odometry as input sensor
    OdometrySensor* odometrySensor = new OdometrySensor(sensorFrameQueue);
    positionEstimator.addSensor(odometrySensor);

    ros::spin();

    // Dispose of everything allocated
    delete odometrySensor;
    delete sensorFrameQueue;

    return 0;
}
