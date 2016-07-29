/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef IMUSENSOR_HPP_
#define IMUSENSOR_HPP_

#include <ros/ros.h>

#include <srslib_framework/filter/Sensor.hpp>
#include <srslib_framework/robotics/Imu.hpp>

namespace srs {

template<unsigned int STATE_SIZE = 5, int TYPE = CV_64F>
class ImuSensor :
    public Sensor<STATE_SIZE, TYPE>
{
public:
    typedef typename Sensor<STATE_SIZE, TYPE>::BaseType BaseType;

    ImuSensor() :
        Sensor<STATE_SIZE, TYPE>(),
        currentData_(Imu<BaseType>::INVALID)
    {
        reset();
    }

    ImuSensor(cv::Mat R) :
        Sensor<STATE_SIZE, TYPE>(R),
        currentData_(Imu<BaseType>::INVALID)
    {
        reset();
    }

    virtual ~ImuSensor()
    {}

    virtual cv::Mat getCurrentData();

    Imu<BaseType> getImu() const
    {
        return currentData_;
    }

    void reset()
    {
        currentData_ = Imu<BaseType>::INVALID;
        Sensor<STATE_SIZE, TYPE>::setNewData(false);
    }

    void set(Imu<BaseType> newImu)
    {
        if (Sensor<STATE_SIZE, TYPE>::isEnabled())
        {
            currentData_ = newImu;
            Sensor<STATE_SIZE, TYPE>::setNewData(true);
        }
    }

    virtual cv::Mat H(const cv::Mat stateVector);

private:
    Imu<BaseType> currentData_;
};

} // namespace srs

#include <tap/sensor_frame/imu/ImuSensor.cpp>

#endif // IMUSENSOR_HPP_
