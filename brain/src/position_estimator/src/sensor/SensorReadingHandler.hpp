/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SENSORREADINGHANDLER_HPP_
#define SENSORREADINGHANDLER_HPP_

#include "MeasurementQueue.hpp"

namespace srs {

class SensorReadingHandler
{
public:
    SensorReadingHandler(const MeasurementQueue<>* queue) :
        queue_(queue)
    {}

    ~SensorReadingHandler()
    {}

    const MeasurementQueue<>* getQueue() const
    {
        return queue_;
    }

private:
    const MeasurementQueue<>* queue_;
};

} // namespace srs

#endif // SENSORREADINGHANDLER_HPP_
