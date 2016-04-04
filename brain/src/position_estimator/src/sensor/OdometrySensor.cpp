#include "OdometrySensor.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////
OdometrySensor::OdometrySensor(const SensorFrameQueue* queue) :
    Sensor(queue)
{
    ROS_INFO_STREAM("Odometry sensor registered");

    ros::NodeHandle rosNodeHandle;

    rosSubscriber_ = rosNodeHandle.subscribe("/sensors/odometry/raw", 100,
        &OdometrySensor::cbMessageReceived, this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OdometrySensor::~OdometrySensor()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OdometrySensor::cbMessageReceived(brain_msgs::RawOdometryConstPtr message)
{
    ROS_INFO_STREAM("Odometry received " << message->left);
}
