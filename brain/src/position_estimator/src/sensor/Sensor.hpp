/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SENSOR_HPP_
#define SENSOR_HPP_

#include <framework/SensorReadingHandler.hpp>
#include <framework/SensorFrameQueue.hpp>

namespace sixrs {

class Sensor :
    public SensorReadingHandler
{
public:
    Sensor(std::string name, const SensorFrameQueue* queue) :
        SensorReadingHandler(queue),
        name_(name)
    {}

    ~Sensor()
    {}

    std::string getName() const;

private:
    std::string name_;
};

} // namespace sixrs

#endif  // SENSOR_HPP_
