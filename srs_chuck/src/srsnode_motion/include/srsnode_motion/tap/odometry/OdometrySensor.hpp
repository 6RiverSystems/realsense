/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ODOMETRYSENSOR_HPP_
#define ODOMETRYSENSOR_HPP_

#include <ros/ros.h>

#include <srslib_framework/filter/Sensor.hpp>
#include <srslib_framework/robotics/Odometry.hpp>

namespace srs {

template<unsigned int STATE_SIZE = 5, int TYPE = CV_64F>
class OdometrySensor :
    public Sensor<STATE_SIZE, TYPE>
{
public:
    // Standard deviation values for the odometry sensor
    constexpr static double ERROR_LINEAR_VELOCITY = 0.004; // [m/s]
    constexpr static double ERROR_ANGULAR_VELOCITY = 0.004; // [m/s]

    typedef typename Sensor<STATE_SIZE, TYPE>::BaseType BaseType;

    OdometrySensor() :
        Sensor<STATE_SIZE, TYPE>(cv::Mat::diag(R)),
        currentData_(Odometry<BaseType>(0, 0, 0))
    {
        reset();
    }

    virtual ~OdometrySensor()
    {}

    virtual cv::Mat getCurrentData();

    Odometry<BaseType> getOdometry()
    {
        Sensor<STATE_SIZE, TYPE>::setNewData(false);
        return currentData_;
    }

    void reset()
    {
        currentData_ = Odometry<BaseType>(0, 0, 0);
        Sensor<STATE_SIZE, TYPE>::setNewData(false);
    }

    void set(double arrivalTime, BaseType linear, BaseType angular)
    {
        if (Sensor<STATE_SIZE, TYPE>::isEnabled())
        {
            ROS_DEBUG_STREAM_NAMED("OdometrySensor", "l:" << linear << ", a: " << angular);

            currentData_ = Odometry<BaseType>(arrivalTime, linear, angular);
            Sensor<STATE_SIZE, TYPE>::setNewData(true);
        }
    }

    virtual cv::Mat H(const cv::Mat stateVector);

private:
    const static cv::Mat R;
    Odometry<BaseType> currentData_;
};

} // namespace srs

#include <tap/odometry/OdometrySensor.cpp>

#endif // ODOMETRYSENSOR_HPP_
