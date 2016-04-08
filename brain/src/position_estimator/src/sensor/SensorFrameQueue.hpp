/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SENSORFRAMEQUEUE_HPP_
#define SENSORFRAMEQUEUE_HPP_

#include <platform/ThreadSafeQueue.hpp>
#include <filter/Measurement.hpp>

namespace srs {

class SensorFrameQueue
{
public:
    SensorFrameQueue()
    {}

    ~SensorFrameQueue()
    {}

    void push(Measurement* reading);

private:
    ThreadSafeQueue<Measurement*> queue_;
};

} // namespace srs

#endif // SENSORFRAMEQUEUE_HPP_
