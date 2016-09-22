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
    typedef typename Sensor<STATE_SIZE, TYPE>::BaseType BaseType;

    OdometrySensor() :
        Sensor<STATE_SIZE, TYPE>(),
        currentData_(Odometry<BaseType>::ZERO)
    {
        reset();
    }

    OdometrySensor(cv::Mat R) :
        Sensor<STATE_SIZE, TYPE>(R),
        currentData_(Odometry<BaseType>::INVALID)
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
        currentData_ = Odometry<BaseType>::ZERO;
        Sensor<STATE_SIZE, TYPE>::setNewData(false);
    }

    void set(Odometry<BaseType> newOdometry)
    {
        if (Sensor<STATE_SIZE, TYPE>::isEnabled())
        {
            currentData_ = newOdometry;
            Sensor<STATE_SIZE, TYPE>::setNewData(true);
        }
    }

    virtual cv::Mat H(const cv::Mat stateVector);

private:
    Odometry<BaseType> currentData_;
};

} // namespace srs

#include <tap/sensor_frame/odometry/OdometrySensor.cpp>

#endif // ODOMETRYSENSOR_HPP_