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

#include <filter/ukf/UnscentedKalmanFilter.hpp>

namespace sixrs {

class PositionEstimator :
    public SensorReadingHandler
{
public:
    PositionEstimator(const SensorFrameQueue* queue);
    ~PositionEstimator();

    void addSensor(const Sensor* newSensor);
    void run();

private:
    const static unsigned int REFRESH_RATE_HZ;
    const static unsigned int STATE_VECTOR_SIZE;
    const static double ALPHA;
    const static double BETA;

    std::vector<const Sensor*> sensors_;

    UnscentedKalmanFilter<CV_64F> ukf_;
};

} // namespace sixrs

#endif  // POSITIONESTIMATOR_HPP_
