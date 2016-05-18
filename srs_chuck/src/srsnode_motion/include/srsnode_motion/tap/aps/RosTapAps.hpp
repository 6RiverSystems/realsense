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

#include <srslib_framework/math/Time.hpp>
#include <srslib_framework/Aps.h>
#include <srslib_framework/ros/RosTap.hpp>

#include <srsnode_motion/tap/aps/ApsSensor.hpp>

namespace srs {

class RosTapAps :
    public RosTap
{
public:
    typedef typename ApsSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>::BaseType BaseType;

    RosTapAps(ros::NodeHandle rosHandle) :
        RosTap(rosHandle, "Absolute Positioning System Tap")
    {
        sensor_ = new ApsSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>();
    }

    ~RosTapAps()
    {
        disconnectTap();
        delete sensor_;
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
    }

    void set(double arrivalTime, BaseType x, BaseType y, BaseType yaw)
    {
        sensor_->set(arrivalTime, x, y, yaw);
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/sensors/aps/raw", 100,
            &RosTapAps::onAps, this);

        return true;
    }

private:
    void onAps(srslib_framework::ApsConstPtr message)
    {
        set(Time::time2number(message->header.stamp),
            static_cast<double>(message->x),
            static_cast<double>(message->y),
            static_cast<double>(message->yaw));
    }

    ApsSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* sensor_;
};

} // namespace srs

#endif // ROSAPS_HPP_
