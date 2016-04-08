/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SENSOR_HPP_
#define SENSOR_HPP_

#include <string>
using namespace std;

#include <filter/Measurement.hpp>

#include "SensorReadingHandler.hpp"
#include "SensorFrameQueue.hpp"

namespace srs {

class Sensor :
    public SensorReadingHandler
{
public:
    Sensor(string name, const SensorFrameQueue* queue);
    virtual ~Sensor();

    virtual Measurement getCurrentData() const = 0;
    string getName() const;

private:
    string name_;
};

} // namespace srs

#endif  // SENSOR_HPP_
