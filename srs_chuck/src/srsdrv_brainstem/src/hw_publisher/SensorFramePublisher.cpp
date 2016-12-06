#include <srsdrv_brainstem/hw_publisher/SensorFramePublisher.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

SensorFramePublisher::SensorFramePublisher(string nameSpace) :
	rosNodeHandle_(nameSpace)
{
	imuPublisher_ = rosNodeHandle_.advertise<srslib_framework::Imu>(
		ChuckTopics::driver::BRAINSTEM_IMU, 1, true);

	odometryPublisher_ = rosNodeHandle_.advertise<srslib_framework::Odometry>(
		ChuckTopics::driver::BRAINSTEM_ODOMETRY_COUNT, 1, true);

	sensorFramePublisher_ = rosNodeHandle_.advertise<srslib_framework::SensorFrame>(
		ChuckTopics::driver::BRAINSTEM_SENSOR_FRAME, 1, true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


void SensorFramePublisher::publishImu(srslib_framework::Imu& imuMsg)
{
	imuPublisher_.publish(imuMsg);
}

void SensorFramePublisher::publishOdometry(srslib_framework::Odometry& odometryMsg)
{
	odometryPublisher_.publish(odometryMsg);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SensorFramePublisher::publishSensorFrame(srslib_framework::SensorFrame& sensorFrameMsg)
{
	sensorFramePublisher_.publish(sensorFrameMsg);
}

} // namespace srs
