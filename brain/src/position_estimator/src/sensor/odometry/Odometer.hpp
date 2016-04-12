/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ODOMETRYSENSOR_HPP_
#define ODOMETRYSENSOR_HPP_

#include <ros/ros.h>

#include <brain_msgs/RawOdometry.h>

#include <filter/Sensor.hpp>

#include "Odometry.hpp"

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
        currentData_(this, 0, 0, 0)
    {
        reset();
    }

    virtual ~Odometer()
    {}

    Measurement<STATE_SIZE, TYPE> getCurrentData() const
    {
        return currentData_;
    }

    void push_back(uint32_t arrivalTime, uint16_t left, uint16_t right)
    {
        currentData_ = Odometry<STATE_SIZE, TYPE>(this, arrivalTime, left, right);
        Sensor<STATE_SIZE, TYPE>::setNewData(true);
    }

    void reset()
    {
        currentData_ = Odometry<STATE_SIZE, TYPE>(this, 0, 0, 0);
        Sensor<STATE_SIZE, TYPE>::setNewData(false);
    }

    virtual cv::Mat transform2State(Measurement<STATE_SIZE, TYPE>* const measurement);
    virtual cv::Mat transformWithH(const cv::Mat state);

private:
    const static cv::Mat R;

    BaseType wheelsRatio_;
    Odometry<STATE_SIZE, TYPE> currentData_;
};

} // namespace srs

#include "Odometer.cpp"

#endif // ODOMETRYSENSOR_HPP_
