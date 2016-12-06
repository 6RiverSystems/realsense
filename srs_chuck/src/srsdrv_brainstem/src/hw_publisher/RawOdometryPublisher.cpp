#include <srsdrv_brainstem/hw_publisher/RawOdometryPublisher.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

RawOdometryPublisher::RawOdometryPublisher(string nameSpace) :
	rosNodeHandle_(nameSpace)
{
	publisher_ = rosNodeHandle_.advertise<srslib_framework::OdometryRPM>(
		ChuckTopics::driver::BRAINSTEM_RAW_ODOMETRY, 1, true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void RawOdometryPublisher::publishRawOdometry(srslib_framework::OdometryRPM& rawOdometryMsg)
{
	publisher_.publish(rawOdometryMsg);
}

} // namespace srs
