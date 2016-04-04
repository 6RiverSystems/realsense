/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef POSITIONESTIMATOR_HPP_
#define POSITIONESTIMATOR_HPP_

#include <vector>

#include <framework/SensorReadingHandler.hpp>
#include <framework/SensorFrameQueue.hpp>
#include <sensor/Sensor.hpp>

class PositionEstimator :
    public SensorReadingHandler
{
public:
    PositionEstimator(const SensorFrameQueue* queue);
    ~PositionEstimator();

    void addSensor(const Sensor* newSensor);

private:
    std::vector<const Sensor*> sensors_;
};

#endif  // POSITIONESTIMATOR_HPP_
