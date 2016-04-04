/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef SENSORREADINGHANDLER_HPP_
#define SENSORREADINGHANDLER_HPP_

#include "SensorFrameQueue.hpp"

class SensorReadingHandler
{
public:
    SensorReadingHandler(const SensorFrameQueue* queue) :
        queue_(queue)
    {}

    ~SensorReadingHandler()
    {}

    const SensorFrameQueue* getQueue() const
    {
        return queue_;
    }

private:
    const SensorFrameQueue* queue_;
};

#endif // SENSORREADINGHANDLER_HPP_
