#include <srsdrv_brainstem/hw_message/SensorFrameHandler.hpp>

#include <srslib_framework/Imu.h>
#include <srslib_framework/SensorFrame.h>
#include <srslib_framework/Velocity.h>
#include <srslib_framework/Odometry.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/math/VelocityMath.hpp>
#include <srslib_framework/ros/message/VelocityMessageFactory.hpp>
#include <srslib_framework/ros/message/ImuMessageFactory.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

SensorFrameHandler::SensorFrameHandler(ImuCallbackFn imuCallback, OdometryCallbackFn odometryCallback,
    SensorFrameCallbackFn sensorFrameCallback) :
    HardwareMessageHandler(BRAIN_STEM_MSG::SENSOR_FRAME),
    lastHwSensorFrameTime_(0),
    lastRosSensorFrameTime_(ros::Time::now()),
    currentOdometry_(Odometry<>::ZERO),
	imuCallback_(imuCallback),
	odometryCallback_(odometryCallback),
	sensorFrameCallback_(sensorFrameCallback)
{

}

int32_t calculateOdometryDiff(uint32_t current, uint32_t& last)
{
	int32_t diff = current - last;
	bool dir = true;

	// Determine direction and delta from previous count
	if(current >= last)
	{
		dir = true;
		diff = current - last;
	}
	else
	{
		dir = false;
		diff = last - current;
	}

	// Fix for quadrature rollover
	if (diff > (1<<15) )
	{
		// Invert Direction
		dir = dir == true ? false : true;

		// adjust movement
		diff = (1<<16)-diff;
	}

	last = current;

	return dir ? diff : -diff;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool SensorFrameHandler::receiveMessage(ros::Time currentTime, HardwareMessage& msg)
{
    MsgSensorFrame sensorData = msg.read<MsgSensorFrame>();

    ros::Time internalTime = currentTime;
	bool timeSliceExpired = TimeMath::isTimeElapsed(OUT_OF_SYNC_TIMEOUT,
		lastRosSensorFrameTime_, internalTime);

	if (timeSliceExpired)
	{
		ROS_ERROR_STREAM_NAMED("sensor_frame",
			"Time-stamp out of range: " <<
			"diff: " << (internalTime.toSec() - lastRosSensorFrameTime_.toSec()) <<
			" last: " << lastRosSensorFrameTime_.toSec() <<
			" current: " << internalTime.toSec());
	}

	// Produce the current robot velocity from the odometry data
	Velocity<> odometryVelocity = Velocity<>(
		sensorData.linear_velocity,
		sensorData.angular_velocity);

	// If the last time the handler received the data is greater than the specified
	// time-out or the robot is standing still, set the arrival time of the data
	// to the current time minus the transmission delay (in this case of the serial port)
	if (timeSliceExpired || VelocityMath::equal<double>(odometryVelocity, Velocity<>::ZERO))
	{
		internalTime = currentTime - ros::Duration(0, SERIAL_TRANSMIT_DELAY);
	}
	else
	{
		// Otherwise calculate the arrival time of the data based on the
		// ROS time of the last sensor frame plus the difference between
		// the two hardware time-stamps
		double deltaTimeSlice = (static_cast<double>(sensorData.timestamp) -
			lastHwSensorFrameTime_) / 1000.0;

		internalTime = lastRosSensorFrameTime_+ ros::Duration(deltaTimeSlice);
	}

	static uint32_t s_leftWheelCount = sensorData.left_wheel_count;
	static uint32_t s_rightWheelCount = sensorData.right_wheel_count;

	int32_t s_leftWheelDiff = calculateOdometryDiff(sensorData.left_wheel_count, s_leftWheelCount);
	int32_t s_rightWheelDiff = calculateOdometryDiff(sensorData.right_wheel_count, s_rightWheelCount);

	// Store the time for the next sensor frame
	lastRosSensorFrameTime_ = internalTime;
	lastHwSensorFrameTime_ = static_cast<double>(sensorData.timestamp);

	// Generate the current odometry value
	double currentTimeSec = internalTime.toSec();
	currentOdometry_ = Odometry<>(Velocity<>(currentTimeSec, odometryVelocity));

	srslib_framework::SensorFrame sensorFrameMsg;

	if (msg.checkBufferSize<MsgImu>())
	{
		MsgImu imuData = msg.read<MsgImu>();

		currentImu_ = Imu<>(currentTimeSec,
			AngleMath::deg2Rad<double>(imuData.yaw),
			AngleMath::deg2Rad<double>(imuData.pitch),
			AngleMath::deg2Rad<double>(imuData.roll),
			imuData.yawRot,
			imuData.pitchRot,
			imuData.rollRot);

		sensorFrameMsg.imu.yaw = imuData.yaw;
		sensorFrameMsg.imu.pitch = imuData.pitch;
		sensorFrameMsg.imu.roll = imuData.roll;
		sensorFrameMsg.imu.yawRot = imuData.yawRot;
		sensorFrameMsg.imu.pitchRot = imuData.pitchRot;
		sensorFrameMsg.imu.rollRot = imuData.rollRot;

		srslib_framework::Imu imuMsg = ImuMessageFactory::imu2Msg(currentImu_);
		imuCallback_(imuMsg);
	}

	sensorFrameMsg.header.stamp = lastRosSensorFrameTime_;
	sensorFrameMsg.odometry = VelocityMessageFactory::velocity2Msg(currentOdometry_.velocity);
	sensorFrameCallback_(sensorFrameMsg);

	srslib_framework::Odometry odometryMsg;
	odometryMsg.left_wheel = s_leftWheelDiff;
	odometryMsg.right_wheel = s_rightWheelDiff;
	odometryMsg.header.stamp = lastRosSensorFrameTime_;
	odometryCallback_(odometryMsg);

	return true;
}

} // namespace srs
