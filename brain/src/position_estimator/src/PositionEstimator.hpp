/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef POSITIONESTIMATOR_HPP_
#define POSITIONESTIMATOR_HPP_

#include <vector>
using namespace std;

#include <framework/RosSensor.hpp>

#include <sensor/SensorReadingHandler.hpp>
#include <sensor/MeasurementQueue.hpp>

#include "Robot.hpp"

#include <filter/ukf/UnscentedKalmanFilter.hpp>

// Preprocessor-level definition of the size of the state vector
// in the estimator
#define STATIC_STATE_VECTOR_SIZE 5

namespace srs {

class PositionEstimator
{
public:
    PositionEstimator();
    ~PositionEstimator();

    void addSensor(const RosSensor* newSensor);

    void run();

private:
    const static unsigned int REFRESH_RATE_HZ;
    const static unsigned int STATE_VECTOR_SIZE;
    const static double ALPHA;
    const static double BETA;

    Robot<STATIC_STATE_VECTOR_SIZE> robot_;
    vector<const RosSensor*> sensors_;

    UnscentedKalmanFilter<STATIC_STATE_VECTOR_SIZE> ukf_;
};

} // namespace srs

#endif  // POSITIONESTIMATOR_HPP_
