/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPSENSORFRAME_HPP_
#define ROSTAPSENSORFRAME_HPP_

#include <ros/ros.h>

#include <srslib_framework/SensorFrame.h>

#include <srslib_framework/ros/tap/RosTap.hpp>
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

        currentRawImu_ = Imu<>::ZERO;

        imuDeltaYaw_ = 0.0;

        imuYawSumCosine_ = 0.0;
        imuYawSumSine_ = 0.0;
        accumulateYaws_ = false;
    }

    ~RosTapSensorFrame()
    {
        disconnectTap();
        delete sensorImu_;
    }

    void addTrueYaw(double trueYaw)
    {
        if (accumulateYaws_)
        {
            imuYawSumCosine_ += cos(trueYaw);
            imuYawSumSine_ += sin(trueYaw);

            double averageAngle = atan2(imuYawSumSine_, imuYawSumCosine_);
            setTrueYaw(averageAngle);
        }
    }

    Imu<> getCalibratedImu() const
    {
        return sensorImu_->getImu();
    }

    Imu<> getRawImu()
    {
        RosTap::setNewData(false);
        return currentRawImu_;
    }

    Odometry<> getOdometry() const
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

    bool newImuRawDataAvailable() const
    {
        return newDataAvailable();
    }

    bool newImuCalibratedDataAvailable() const
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

    void setCalibratedImu(Imu<> imu)
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

    void setRawImu(Imu<> imu)
    {
        currentRawImu_ = imu;
        setNewData(true);
    }

    void setTrueYaw(double newValue)
    {
        imuDeltaYaw_ = AngleMath::normalizeRad<double>(newValue - currentRawImu_.yaw);
    }

    void stopAccumulatingYaw()
    {
        accumulateYaws_ = false;
    }

    void startAccumulatingYaw()
    {
        if (!accumulateYaws_)
        {
            accumulateYaws_ = true;
            imuYawSumCosine_ = 0.0;
            imuYawSumSine_ = 0.0;
        }
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
        setRawImu(ImuMessageFactory::msg2Imu(message->imu));
        setOdometry(Odometry<>(VelocityMessageFactory::msg2Velocity(message->odometry)));

        Imu<> calibratedImu = Imu<>(
            currentRawImu_.arrivalTime,
            AngleMath::normalizeRad<double>(currentRawImu_.yaw + imuDeltaYaw_),
            0.0, 0.0,
            currentRawImu_.yawRot, 0.0, 0.0);
        setCalibratedImu(calibratedImu);
    }

    Imu<> currentRawImu_;
    ImuSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* sensorImu_;
    OdometrySensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* sensorOdometry_;

    double imuYawSumCosine_;
    double imuYawSumSine_;

    double imuDeltaYaw_;
    bool accumulateYaws_;
};

} // namespace srs

#endif // ROSTAPSENSORFRAME_HPP_
