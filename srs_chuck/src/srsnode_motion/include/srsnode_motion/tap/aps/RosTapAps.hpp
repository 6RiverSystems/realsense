/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSAPS_HPP_
#define ROSAPS_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/ros/RosTap.hpp>
#include <srsnode_motion/tap/aps/ApsSensor.hpp>

#include <srslib_framework/MsgPose.h>

namespace srs {

class RosTapAps :
    public RosTap
{
public:
    typedef typename ApsSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>::BaseType BaseType;

    RosTapAps() :
        RosTap("Absolute Positioning System Tap")
    {
        sensor_ = new ApsSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>();
    }

    ~RosTapAps()
    {
        disconnectTap();
        delete sensor_;
    }

    Pose<> getPose()
    {
        return sensor_->getPose();
    }

    ApsSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* getSensor() const
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

    void set(double arrivalTime, BaseType x, BaseType y, BaseType theta)
    {
        sensor_->set(arrivalTime, x, y, theta);

        if (!sensor_->isEnabled())
        {
            ROS_DEBUG_STREAM_NAMED("ros_tap_aps", "APS disabled. Ignoring readings.");
        }

        setNewData(sensor_->newDataAvailable());
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/internal/sensors/aps/raw", 100,
            &RosTapAps::onAps, this);

        return true;
    }

private:
    void onAps(const srslib_framework::MsgPoseConstPtr& message)
    {
        set(TimeMath::time2number(message->header.stamp),
            static_cast<double>(message->x),
            static_cast<double>(message->y),
            static_cast<double>(AngleMath::deg2rad(message->theta)));
    }

    ApsSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* sensor_;
};

} // namespace srs

#endif // ROSAPS_HPP_
