/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SENSORREADING_HPP_
#define SENSORREADING_HPP_

namespace sixrs {

/**
 * Interface for the sensor frame queue.
 */
struct SensorReading
{
    SensorReading()
    {}

    virtual ~SensorReading()
    {}
};

} // namespace sixrs

#endif // SENSORREADING_HPP_
