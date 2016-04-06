#include "OdometrySensor.hpp"

namespace sixrs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
OdometrySensor::OdometrySensor(const SensorFrameQueue* queue) :
    Sensor("OdometrySensor", queue),
    currentData_(0, 0, 0),
    newData_(false)
{
    ros::NodeHandle rosNodeHandle;

    rosSubscriber_ = rosNodeHandle.subscribe("/sensors/odometry/raw", 100,
        &OdometrySensor::cbMessageReceived, this);

    reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OdometrySensor::~OdometrySensor()
{
    rosSubscriber_.shutdown();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool OdometrySensor::newDataAvailable() const
{
    return newData_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OdometrySensor::reset()
{
    currentData_ = OdometryReading(0, 0, 0);

    newData_ = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void OdometrySensor::cbMessageReceived(brain_msgs::RawOdometryConstPtr message)
{
    // Store the most updated odometry information
    currentData_ = OdometryReading(
        message->header.stamp.nsec,
        message->left,
        message->right);

    newData_ = true;
}

} // namespace sixrs
