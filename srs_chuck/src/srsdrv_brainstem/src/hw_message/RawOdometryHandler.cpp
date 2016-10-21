#include <srsdrv_brainstem/hw_message/RawOdometryHandler.hpp>

#include <srslib_framework/OdometryRPM.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/math/VelocityMath.hpp>
#include <srslib_framework/ros/message/VelocityMessageFactory.hpp>
#include <srslib_framework/ros/message/ImuMessageFactory.hpp>

namespace srs {

const string RawOdometryHandler::TOPIC_RAW_ODOMETRY = "/internal/sensors/odometry/raw";

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

RawOdometryHandler::RawOdometryHandler() :
    BrainstemMessageHandler(RAW_ODOMETRY_KEY),
    lastHwSensorFrameTime_(0),
    lastRosSensorFrameTime_(ros::Time::now())
{
	pubOdometry_ = rosNodeHandle_.advertise<srslib_framework::OdometryRPM>(
		TOPIC_RAW_ODOMETRY , 100);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void RawOdometryHandler::receiveData(ros::Time currentTime, vector<char>& buffer)
{
	RawOdometryData* odometryRPMData = reinterpret_cast<RawOdometryData*>(buffer.data());

	ros::Time internalTime = currentTime;
	bool timeSliceExpired = TimeMath::isTimeElapsed(OUT_OF_SYNC_TIMEOUT,
	        lastRosSensorFrameTime_, internalTime);

	if (timeSliceExpired)
	{
	    ROS_ERROR_STREAM_NAMED("odometry_frame",
	        "Time-stamp out of range: " <<
	        "diff: " << (internalTime.toSec() - lastRosSensorFrameTime_.toSec()) <<
	        " last: " << lastRosSensorFrameTime_.toSec() <<
	        " current: " << internalTime.toSec());
	}

	bool isRobotStatic = true;
	if (fabs(odometryRPMData->rpm_left_wheel) > 0.00001 || fabs(odometryRPMData->rpm_right_wheel) > 0.00001)
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
	    double deltaTimeSlice = (static_cast<double>(odometryRPMData->timestamp) -
	        lastHwSensorFrameTime_) / 1000.0;

	    internalTime = lastRosSensorFrameTime_+ ros::Duration(deltaTimeSlice);
	}

	lastRosSensorFrameTime_ = internalTime;
	lastHwSensorFrameTime_ = static_cast<double>(odometryRPMData->timestamp);

	publishOdometry(odometryRPMData->rpm_left_wheel, odometryRPMData->rpm_right_wheel);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

void RawOdometryHandler::publishOdometry(float leftWheelRPM, float rightWheelRPM)
{
	srslib_framework::OdometryRPM message;
	message.header.stamp = lastRosSensorFrameTime_;

	message.left_wheel_rpm = leftWheelRPM;
	message.right_wheel_rpm = rightWheelRPM;

	pubOdometry_.publish(message);
}

} // namespace srs
