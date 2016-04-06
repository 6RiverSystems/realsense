/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SENSORFRAMEQUEUE_HPP_
#define SENSORFRAMEQUEUE_HPP_

#include <platform/ThreadSafeQueue.hpp>
#include "SensorReading.hpp"

namespace sixrs {

class SensorFrameQueue
{
public:
    SensorFrameQueue()
    {}

    ~SensorFrameQueue()
    {}

    void push(SensorReading* reading);

private:
    ThreadSafeQueue<SensorReading*> queue_;
};

} // namespace sixrs

#endif // SENSORFRAMEQUEUE_HPP_
