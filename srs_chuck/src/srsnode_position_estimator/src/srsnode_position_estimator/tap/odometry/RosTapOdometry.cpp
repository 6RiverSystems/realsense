#include <srsnode_position_estimator/tap/odometry/RosTapOdometry.hpp>

#include <srslib_framework/math/Time.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
bool RosTapOdometry::connect()
{
    rosSubscriber_ = rosNodeHandle_.subscribe("/sensors/odometry/raw", 100,
        &RosTapOdometry::onSensorsOdometryRaw, this);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void RosTapOdometry::onSensorsOdometryRaw(geometry_msgs::TwistStampedConstPtr message)
{
    set(Time::time2number(message->header.stamp),
        static_cast<double>(message->twist.linear.x),
        static_cast<double>(message->twist.angular.z));
}

} // namespace srs
