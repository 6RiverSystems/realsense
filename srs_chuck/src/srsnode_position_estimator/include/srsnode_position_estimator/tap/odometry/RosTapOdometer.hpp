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

#include <srsnode_position_estimator/Configuration.hpp>

#include "Odometer.hpp"

namespace srs {

class RosTapOdometer :
    public RosTap
{
public:
    typedef typename Odometer<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>::BaseType BaseType;

    RosTapOdometer() :
        RosTap("Odometer")
    {
        sensor_ = new Odometer<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>();
    }

    ~RosTapOdometer()
    {
        disconnectTap();
        delete sensor_;
    }

    Odometer<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* getSensor() const
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
    bool connect();

private:
    void onSensorsOdometryRaw(geometry_msgs::TwistStampedConstPtr message);

    Odometer<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* sensor_;
};

} // namespace srs

#endif // ROSODOMETER_HPP_
