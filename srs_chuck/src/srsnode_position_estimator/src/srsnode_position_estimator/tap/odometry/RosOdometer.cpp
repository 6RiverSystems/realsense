#include <srsnode_position_estimator/tap/odometry/RosOdometer.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
bool RosOdometer::connect()
{
    rosSubscriber_ = rosNodeHandle_.subscribe("/sensors/odometry/raw", 100,
        &RosOdometer::onSensorsOdometryRaw, this);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void RosOdometer::onSensorsOdometryRaw(geometry_msgs::TwistStampedConstPtr message)
{
    set(message->header.stamp.nsec,
        static_cast<double>(message->twist.linear.x),
        static_cast<double>(message->twist.angular.z));
}

} // namespace srs
