/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <opencv2/opencv.hpp>

#include <framework/Math.hpp>

#include <filter/Process.hpp>

#include "Configuration.hpp"
#include "VelCmd.hpp"

namespace srs {

template<int TYPE = CV_64F>
class Robot : public Process<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>
{
public:
    typedef typename Process<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>::BaseType BaseType;

    Robot() :
        Process<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>(cv::Mat::diag(Q))
    {}

    virtual ~Robot()
    {}

    virtual cv::Mat transformWithAB(const cv::Mat stateVector,
        Command<STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>* const command,
        BaseType dT);

private:
    const static cv::Mat Q;
};

} // namespace srs

#include "Robot.cpp"

#endif // ROBOT_HPP_
