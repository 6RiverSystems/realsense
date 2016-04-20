/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ODOMETRYSENSOR_HPP_
#define ODOMETRYSENSOR_HPP_

#include <ros/ros.h>

#include <filter/Sensor.hpp>

#include "Odometry.hpp"
#include "Configuration.hpp"

namespace srs {

template<unsigned int STATE_SIZE = 5, int TYPE = CV_64F>
class Odometer :
    public Sensor<STATE_SIZE, TYPE>
{
public:
    typedef typename Sensor<STATE_SIZE, TYPE>::BaseType BaseType;

    Odometer(BaseType wheelsDistance) :
        Sensor<STATE_SIZE, TYPE>(cv::Mat::diag(R)),
        wheelsRatio_(BaseType(0.5) / BaseType(wheelsDistance)),
        currentData_(Odometry<BaseType>(0, 0, 0))
    {
        reset();
    }

    virtual ~Odometer()
    {}

    void push_back(uint32_t arrivalTime, BaseType left, BaseType right)
    {
        currentData_ = Odometry<BaseType>(arrivalTime, left, right);
        Sensor<STATE_SIZE, TYPE>::setNewData(true);
    }

    void reset()
    {
        currentData_ = Odometry<BaseType>(0, 0, 0);
        Sensor<STATE_SIZE, TYPE>::setNewData(false);
    }

    virtual cv::Mat getCurrentData();
    virtual cv::Mat transformWithH(const cv::Mat stateVector);

private:
    const static cv::Mat R;

    BaseType wheelsRatio_;
    Odometry<BaseType> currentData_;
};

} // namespace srs

#include "Odometer.cpp"

#endif // ODOMETRYSENSOR_HPP_
