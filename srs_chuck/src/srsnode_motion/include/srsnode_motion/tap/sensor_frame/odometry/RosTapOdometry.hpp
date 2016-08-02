/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPODOMETRY_HPP_
#define ROSTAPODOMETRY_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/math/TimeMath.hpp>

#include <srsnode_motion/Configuration.hpp>

#include <srsnode_motion/tap/sensor_frame/odometry/OdometrySensor.hpp>

namespace srs {

class RosTapOdometry :
    public RosTap
{
public:
    typedef typename OdometrySensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>::BaseType BaseType;

    RosTapOdometry() :
        RosTap("Odometry Tap")
    {
        sensor_ = new OdometrySensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>();
    }

    ~RosTapOdometry()
    {
        disconnectTap();
        delete sensor_;
    }

    Odometry<> getOdometry()
    {
        return sensor_->getOdometry();
    }

    OdometrySensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* getSensor() const
    {
        return sensor_;
    }

    bool newDataAvailable() const
    {
        return sensor_->newDataAvailable();
    }

    void reset()
    {
        sensor_->reset();
        RosTap::reset();
    }

    void set(double arrivalTime, BaseType linear, BaseType angular)
    {
        sensor_->set(arrivalTime, linear, angular);

        if (!sensor_->isEnabled())
        {
            ROS_DEBUG_STREAM_NAMED("ros_tap_odometry", "ODOMETRY disabled. Ignoring readings.");
        }

        setNewData(sensor_->newDataAvailable());
   }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/internal/sensors/odometry/raw", 100,
            &RosTapOdometry::onSensorsOdometryRaw, this);

        return true;
    }

private:
    void onSensorsOdometryRaw(geometry_msgs::TwistStampedConstPtr message)
    {
        set(TimeMath::time2number(message->header.stamp),
            static_cast<double>(message->twist.linear.x),
            static_cast<double>(message->twist.angular.z));
    }

    OdometrySensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* sensor_;
};

} // namespace srs

#endif // ROSTAPODOMETRY_HPP_
