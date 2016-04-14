#include "RosOdometer.hpp"

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
RosOdometer::RosOdometer(string name) :
    RosSensor(name),
    sensor_()
{
    rosSubscriber_ = rosNodeHandle_.subscribe("/sensors/odometry/raw", 100,
        &RosOdometer::cbMessageReceived, this);

    reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
RosOdometer::~RosOdometer()
{
    rosSubscriber_.shutdown();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void RosOdometer::cbMessageReceived(geometry_msgs::TwistStampedConstPtr message)
{
    sensor_->push_back(message->header.stamp.nsec,
        static_cast<double>(message->twist.linear.x),
        static_cast<double>(message->twist.angular.z));
}

} // namespace srs
