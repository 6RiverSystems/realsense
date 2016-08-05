/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef APS_HPP_
#define APS_HPP_

#include <ros/ros.h>

#include <srslib_framework/filter/Sensor.hpp>
#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

template<unsigned int STATE_SIZE = 5, int TYPE = CV_64F>
class ApsSensor :
    public Sensor<STATE_SIZE, TYPE>
{
public:
    typedef typename Sensor<STATE_SIZE, TYPE>::BaseType BaseType;

    ApsSensor() :
        Sensor<STATE_SIZE, TYPE>(),
        currentData_(Pose<BaseType>(0.0, 0.0, 0.0))
    {
        reset();
    }

    virtual ~ApsSensor()
    {}

    virtual cv::Mat getCurrentData();

    Pose<BaseType> getPose()
    {
        return currentData_;
    }

    void reset()
    {
        currentData_ = Pose<BaseType>(0.0, 0.0, 0.0);
        Sensor<STATE_SIZE, TYPE>::setNewData(false);
    }

    void set(double arrivalTime, BaseType x, BaseType y, BaseType theta)
    {
        if (Sensor<STATE_SIZE, TYPE>::isEnabled())
        {
            currentData_ = Pose<BaseType>(arrivalTime, x, y, theta);
            Sensor<STATE_SIZE, TYPE>::setNewData(true);
        }
    }

    virtual cv::Mat H(const cv::Mat stateVector);

private:
    Pose<BaseType> currentData_;
};

} // namespace srs

#include <tap/sensor_frame/aps/ApsSensor.cpp>

#endif // APS_HPP_