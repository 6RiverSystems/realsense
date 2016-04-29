#include <srsnode_position_estimator/tap/odometry/RosTapOdometer.hpp>

#include <srslib_framework/math/Time.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
bool RosTapOdometer::connect()
{
    rosSubscriber_ = rosNodeHandle_.subscribe("/sensors/odometry/raw", 100,
        &RosTapOdometer::onSensorsOdometryRaw, this);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void RosTapOdometer::onSensorsOdometryRaw(geometry_msgs::TwistStampedConstPtr message)
{
//    set(Time::time2number(message->header.stamp),
//        static_cast<double>(message->twist.linear.x),
//        static_cast<double>(message->twist.angular.z));
}

} // namespace srs
