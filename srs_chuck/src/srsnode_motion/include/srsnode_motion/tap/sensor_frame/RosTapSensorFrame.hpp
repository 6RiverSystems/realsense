/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPSENSORFRAME_HPP_
#define ROSTAPSENSORFRAME_HPP_

#include <ros/ros.h>

#include <srslib_framework/SensorFrame.h>

#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/ros/message/ImuMessageFactory.hpp>
#include <srslib_framework/ros/message/VelocityMessageFactory.hpp>

#include <srsnode_motion/tap/sensor_frame/imu/ImuSensor.hpp>
#include <srsnode_motion/tap/sensor_frame/odometry/OdometrySensor.hpp>

namespace srs {

class RosTapSensorFrame :
    public RosTap
{
public:
    RosTapSensorFrame() :
        RosTap("Sensor Frame Tap")
    {
        sensorImu_ = new ImuSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>();
        sensorOdometry_ = new OdometrySensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>();
    }

    ~RosTapSensorFrame()
    {
        disconnectTap();
        delete sensorImu_;
    }

    Imu<> getImu() const
    {
        return sensorImu_->getImu();
    }

    Odometry<> getOdometry()
    {
        return sensorOdometry_->getOdometry();
    }

    ImuSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* getSensorImu() const
    {
        return sensorImu_;
    }

    OdometrySensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* getSensorOdometry() const
    {
        return sensorOdometry_;
    }

    bool newImuDataAvailable() const
    {
        return sensorImu_->newDataAvailable();
    }

    bool newOdometryDataAvailable() const
    {
        return sensorOdometry_->newDataAvailable();
    }

    void reset()
    {
        sensorImu_->reset();
        sensorOdometry_->reset();
        RosTap::reset();
    }

    void setImu(Imu<> imu)
    {
        sensorImu_->set(imu);

        if (!sensorImu_->isEnabled())
        {
            ROS_DEBUG_STREAM_NAMED("ros_tap_imu", "IMU disabled. Ignoring readings.");
        }

        setNewData(sensorImu_->newDataAvailable());
    }

    void setOdometry(Odometry<> newOdometry)
    {
        sensorOdometry_->set(newOdometry);

        if (!sensorOdometry_->isEnabled())
        {
            ROS_DEBUG_STREAM_NAMED("ros_tap_odometry", "ODOMETRY disabled. Ignoring readings.");
        }

        setNewData(sensorOdometry_->newDataAvailable());
   }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/internal/sensors/sensor_frame/raw", 100,
            &RosTapSensorFrame::onSensorFrame, this);

        return true;
    }

private:
    void onSensorFrame(const srslib_framework::SensorFrameConstPtr& message)
    {
        setImu(ImuMessageFactory::msg2Imu(message->imu));
        setOdometry(Odometry<>(VelocityMessageFactory::msg2Velocity(message->odometry)));
    }

    ImuSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* sensorImu_;
    OdometrySensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* sensorOdometry_;
};

} // namespace srs

#endif // ROSTAPSENSORFRAME_HPP_
