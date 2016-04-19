/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSODOMETRYSENSOR_HPP_
#define ROSODOMETRYSENSOR_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>

#include <framework/RosSensor.hpp>
#include <Configuration.hpp>
#include "Odometer.hpp"

namespace srs {

class RosOdometer :
    public RosSensor
{
public:
    RosOdometer(string name);
    ~RosOdometer();

    virtual bool newDataAvailable() const
    {
        return sensor_.newDataAvailable();
    }

    virtual void reset()
    {
        sensor_.reset();
    }

private:
    void cbOdometryReceived(geometry_msgs::TwistStampedConstPtr message);

    Odometer<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE> sensor_;
};

} // namespace srs

#endif // ROSODOMETRYSENSOR_HPP_
