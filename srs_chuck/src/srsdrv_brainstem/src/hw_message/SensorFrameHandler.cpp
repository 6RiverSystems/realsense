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

    // Store the time for the next sensor frame
    lastRosSensorFrameTime_ = internalTime;
    lastHwSensorFrameTime_ = static_cast<double>(sensorData->timestamp);

    // Generate the current odometry value
    double currentTimeSec = internalTime.toSec();
    currentOdometry_ = Odometry<>(Velocity<>(currentTimeSec, odometryVelocity));
    currentImu_ = Imu<>(currentTimeSec,
        AngleMath::deg2rad<double>(sensorData->yaw),
        AngleMath::deg2rad<double>(sensorData->pitch),
        AngleMath::deg2rad<double>(sensorData->roll),
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



// m_odometryRawPublisher = m_rosNodeHandle.advertise<geometry_msgs::TwistStamped>( ODOMETRY_TOPIC, 100 );


// message.header.stamp = m_rosOdomTime;
//    odometry.twist.linear.x = fLinearVelocity;
//    odometry.twist.angular.z = fAngularVelocity;
//
//    m_odometryRawPublisher.publish( odometry );


//void BrainStem::OnOdometryChanged( uint32_t dwTimeStamp, float fLinearVelocity, float fAngularVelocity )
//{
//    ros::Time currentTime = ros::Time::now( );
//
//    bool bInvalidTime = ( currentTime.toSec( ) - m_rosOdomTime.toSec( ) ) > 0.15;
//
//    if( !m_rosOdomTime.isZero( ) &&
//        bInvalidTime )
//    {
//        ROS_ERROR( "Timestamp out of range and resynced (possible communication problem) (diff: %f): odom: %f, ros: %f",
//            currentTime.toSec( ) - m_rosOdomTime.toSec( ), m_rosOdomTime.toSec( ), currentTime.toSec( ) );
//    }
//
//    if (bInvalidTime ||
//           (BasicMath::equal<float>(fLinearVelocity, 0.0f, 0.00001) &&
//            BasicMath::equal<float>(fAngularVelocity, 0.0f, 0.00001)))
//    {
//        // Reset our time basis to account for any drift
//        m_rosOdomTime = currentTime - ros::Duration( 0, 1200000 );
//    }
//    else
//    {
//        double dfOdomTimeDelta = (double)(dwTimeStamp - m_dwLastOdomTime) / 1000.0f;
//
//        ROS_DEBUG_NAMED( "odometry", "Odometry (%f): %f, %f", dfOdomTimeDelta, fLinearVelocity, fAngularVelocity );
//
//        // Base our time on the realtime clock (brain_stem) since our clock does not match odom info
//        // and our loop is not realtime
//        m_rosOdomTime += ros::Duration( dfOdomTimeDelta );
//    }
//
//    geometry_msgs::TwistStamped odometry;
//    odometry.header.stamp = m_rosOdomTime;
//    odometry.twist.linear.x = fLinearVelocity;
//    odometry.twist.angular.z = fAngularVelocity;
//
//    m_odometryRawPublisher.publish( odometry );
//
//    double dfClockDiff = currentTime.toSec( ) - m_rosOdomTime.toSec( );
//
//    if( dfClockDiff > 0.05 )
//    {
//        ROS_ERROR( "Odometry clock drift: %f", dfClockDiff );
//    }
//
//    m_dwLastOdomTime = dwTimeStamp;
//}
