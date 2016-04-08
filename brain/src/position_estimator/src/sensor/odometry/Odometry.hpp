/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include <filter/Measurement.hpp>

namespace srs {

struct Odometry :
    public Measurement
{
    uint32_t arrivalTime;
    uint16_t left;
    uint16_t right;

    Odometry(uint32_t arrivalTime, uint16_t left, uint16_t right) :
        arrivalTime(arrivalTime),
        left(left),
        right(right)
    {}
};

} // namespace srs

#endif // ODOMETRY_HPP_
