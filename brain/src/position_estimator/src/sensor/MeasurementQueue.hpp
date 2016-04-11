/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MEASUREMENTQUEUE_HPP_
#define MEASUREMENTQUEUE_HPP_

#include <platform/ThreadSafeQueue.hpp>

#include <filter/Measurement.hpp>

namespace srs {

template<unsigned int STATE_SIZE = 5, int TYPE = CV_64F>
class MeasurementQueue
{
public:
    MeasurementQueue()
    {}

    ~MeasurementQueue()
    {}

//    void push(Measurement<STATE_SIZE, TYPE>* reading)
//    {}

private:
//    ThreadSafeQueue<Measurement<STATE_SIZE, TYPE>*> queue_;
};

} // namespace srs

#endif // MEASUREMENTQUEUE_HPP_
