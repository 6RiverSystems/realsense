/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include <opencv2/opencv.hpp>

#include <filter/Measurement.hpp>

namespace srs {

template<unsigned int STATE_SIZE = 5, int TYPE = CV_64F>
struct Odometry :
    public Measurement<STATE_SIZE, TYPE>
{
    uint32_t arrivalTime;
    uint16_t left;
    uint16_t right;

    Odometry(Sensor<STATE_SIZE, TYPE>* sensor,
        uint32_t arrivalTime, uint16_t left, uint16_t right) :
            Measurement<STATE_SIZE, TYPE>(sensor),
            arrivalTime(arrivalTime),
            left(left),
            right(right)
    {}
};

} // namespace srs

#endif // ODOMETRY_HPP_
