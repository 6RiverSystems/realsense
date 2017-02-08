#include "../../include/srsdrv_brainstem/hw_message/OdometryRpmHandler.hpp"

#include <srslib_framework/OdometryRpm.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/ros/message/VelocityMessageFactory.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

OdometryRpmHandler::OdometryRpmHandler(ChannelBrainstemOdometryRpm::Interface& publisher) :
    HardwareMessageHandler(BRAIN_STEM_MSG::RAW_ODOMETRY),
    lastHwOdometryTime_(0),
    lastRosOdometryTime_(ros::Time::now()),
	publisher_(publisher)
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OdometryRpmHandler::receiveMessage(ros::Time currentTime, HardwareMessage& msg)
{
	OdometryRpmData odometryRpmData = msg.read<OdometryRpmData>();

	ros::Time internalTime = currentTime;
	bool timeSliceExpired = TimeMath::isTimeElapsed(OUT_OF_SYNC_TIMEOUT,
			lastRosOdometryTime_, internalTime);

	if (timeSliceExpired)
	{
		/// @todo FIX PRECISION
		ROS_ERROR_STREAM_NAMED("odometry_frame",
			"Time-stamp out of range: " <<
			" diff: " << (internalTime.toSec() - lastRosOdometryTime_.toSec()) <<
			" last: " << lastRosOdometryTime_.toSec() <<
			" current: " << internalTime.toSec());
	}

	constexpr static double RPM_EPSILON = 0.00001;

	bool isRobotStatic = true;
	if (fabs(odometryRpmData.rpm_left_wheel) > RPM_EPSILON || fabs(odometryRpmData.rpm_right_wheel) > RPM_EPSILON)
	{
		isRobotStatic = false;
	}

	// If the last time the handler received the data is greater than the specified
	// time-out or the robot is standing still, set the arrival time of the data
	// to the current time minus the transmission delay (in this case of the serial port)
	if (timeSliceExpired || isRobotStatic)
	{
		internalTime = currentTime - ros::Duration(0, SERIAL_TRANSMIT_DELAY);
	}
	else
	{
		// Otherwise calculate the arrival time of the data based on the
		// ROS time of the last sensor frame plus the difference between
		// the two hardware time-stamps
		double deltaTimeSlice = (static_cast<double>(odometryRpmData.timestamp) -
			lastHwOdometryTime_) / 1000.0;

		internalTime = lastRosOdometryTime_+ ros::Duration(deltaTimeSlice);
	}

	lastRosOdometryTime_ = internalTime;
	lastHwOdometryTime_ = static_cast<double>(odometryRpmData.timestamp);

	srslib_framework::OdometryRpm message;
	message.header.stamp = lastRosOdometryTime_;

//		ROS_DEBUG_NAMED("velocity", "left wheel raw: %f, right wheel raw: %f",
//			odometryRpmData.rpm_left_wheel, odometryRpmData.rpm_right_wheel);

	message.left_wheel_rpm = odometryRpmData.rpm_left_wheel;
	message.right_wheel_rpm = odometryRpmData.rpm_right_wheel;

	publisher_.publish(message);
}

} // namespace srs
