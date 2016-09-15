#include <srsdrv_brainstem/hw_message/SensorFrameHandler.hpp>

#include <srslib_framework/Imu.h>
#include <srslib_framework/SensorFrame.h>
#include <srslib_framework/Velocity.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/math/VelocityMath.hpp>
#include <srslib_framework/ros/message/VelocityMessageFactory.hpp>
#include <srslib_framework/ros/message/ImuMessageFactory.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant definitions

const string SensorFrameHandler::TOPIC_ODOMETRY = "/internal/sensors/odometry/raw";
const string SensorFrameHandler::TOPIC_IMU = "/internal/sensors/imu/raw";
const string SensorFrameHandler::TOPIC_SENSOR_FRAME = "/internal/sensors/sensor_frame/raw";

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

SensorFrameHandler::SensorFrameHandler() :
    BrainstemMessageHandler(SENSOR_FRAME_KEY),
    lastHwSensorFrameTime_(0),
    lastRosSensorFrameTime_(ros::Time::now()),
    currentOdometry_(Odometry<>::ZERO)
{
    pubOdometry_ = rosNodeHandle_.advertise<geometry_msgs::TwistStamped>(
        TOPIC_ODOMETRY, 100);
    pubImu_ = rosNodeHandle_.advertise<srslib_framework::Imu>(
        TOPIC_IMU, 100);
    pubSensorFrame_ = rosNodeHandle_.advertise<srslib_framework::SensorFrame>(
        TOPIC_SENSOR_FRAME, 100);
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
void SensorFrameHandler::receiveData(ros::Time currentTime, vector<char>& buffer)
{
    SensorFrameData* sensorData = reinterpret_cast<SensorFrameData*>(buffer.data());

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
        sensorData->linear_velocity,
        sensorData->angular_velocity);

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
        double deltaTimeSlice = (static_cast<double>(sensorData->timestamp) -
            lastHwSensorFrameTime_) / 1000.0;

        internalTime = lastRosSensorFrameTime_+ ros::Duration(deltaTimeSlice);
    }

    static uint32_t s_leftWheelCount = sensorData->left_wheel_count;
    static uint32_t s_rightWheelCount = sensorData->right_wheel_count;
//
//    static int64_t s_leftWheelTotalCount = 0;
//    static int64_t s_rightWheeTotallCount = 0;

    int32_t s_leftWheelDiff = calculateOdometryDiff(sensorData->left_wheel_count, s_leftWheelCount);
    int32_t s_rightWheelDiff = calculateOdometryDiff(sensorData->right_wheel_count, s_rightWheelCount);

//    s_leftWheelTotalCount += s_leftWheelDiff;
//    s_rightWheeTotallCount += s_rightWheelDiff;

//    ROS_ERROR( "%d, %d", s_leftWheelDiff, s_rightWheelDiff );

    // Store the time for the next sensor frame
    lastRosSensorFrameTime_ = internalTime;
    lastHwSensorFrameTime_ = static_cast<double>(sensorData->timestamp);

    // Generate the current odometry value
    double currentTimeSec = internalTime.toSec();
    currentOdometry_ = Odometry<>(Velocity<>(currentTimeSec, odometryVelocity));
    currentImu_ = Imu<>(currentTimeSec,
        AngleMath::deg2Rad<double>(sensorData->yaw),
        AngleMath::deg2Rad<double>(sensorData->pitch),
        AngleMath::deg2Rad<double>(sensorData->roll),
        sensorData->yawRot,
        sensorData->pitchRot,
        sensorData->rollRot);

    // Publish all the data
    publishSensorFrame();
    publishOdometry();
    publishImu();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void SensorFrameHandler::publishImu()
{
    srslib_framework::Imu message = ImuMessageFactory::imu2Msg(currentImu_);
    pubImu_.publish(message);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SensorFrameHandler::publishOdometry()
{
    geometry_msgs::TwistStamped message = VelocityMessageFactory::velocity2TwistStamped(
        currentOdometry_.velocity);
    message.header.stamp = lastRosSensorFrameTime_;

    pubOdometry_.publish(message);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SensorFrameHandler::publishSensorFrame()
{
    srslib_framework::SensorFrame message;
    message.header.stamp = lastRosSensorFrameTime_;
    message.imu = ImuMessageFactory::imu2Msg(currentImu_);
    message.odometry = VelocityMessageFactory::velocity2Msg(currentOdometry_.velocity);
    pubSensorFrame_.publish(message);
}

} // namespace srs
