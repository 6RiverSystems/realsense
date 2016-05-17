/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSODOMETER_HPP_
#define ROSODOMETER_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/math/Time.hpp>

#include <srsnode_motion/Configuration.hpp>

#include <srsnode_motion/tap/odometry/OdometrySensor.hpp>

namespace srs {

class RosTapOdometry :
    public RosTap
{
public:
    typedef typename OdometrySensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>::BaseType BaseType;

    RosTapOdometry(ros::NodeHandle rosHandle) :
        RosTap(rosHandle, "Odometry Tap")
    {
        sensor_ = new OdometrySensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>();
    }

    ~RosTapOdometry()
    {
        disconnectTap();
        delete sensor_;
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
    }

    void set(double arrivalTime, BaseType linear, BaseType angular)
    {
        sensor_->set(arrivalTime, linear, angular);
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/sensors/odometry/raw", 100,
            &RosTapOdometry::onSensorsOdometryRaw, this);

        return true;
    }

private:
    void onSensorsOdometryRaw(geometry_msgs::TwistStampedConstPtr message)
    {
        set(Time::time2number(message->header.stamp),
            static_cast<double>(message->twist.linear.x),
            static_cast<double>(message->twist.angular.z));
    }

    OdometrySensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* sensor_;
};

} // namespace srs

#endif // ROSODOMETER_HPP_
