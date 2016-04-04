/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef SENSOR_HPP_
#define SENSOR_HPP_

#include <framework/SensorReadingHandler.hpp>
#include <framework/SensorFrameQueue.hpp>

class Sensor :
    public SensorReadingHandler
{
public:
    Sensor(const SensorFrameQueue* queue) :
        SensorReadingHandler(queue)
    {}

    ~Sensor()
    {}

private:
};

#endif  // SENSOR_HPP_
