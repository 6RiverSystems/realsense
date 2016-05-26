/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <opencv2/opencv.hpp>

#include <srslib_framework/math/Math.hpp>
#include <srslib_framework/filter/Process.hpp>
#include <srslib_framework/filter/Command.hpp>

#include <srsnode_motion/Configuration.hpp>

namespace srs {

template<int TYPE = CV_64F>
class Robot : public Process<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>
{
public:
    // Standard deviation values for the robot process
    constexpr static double ERROR_LOCATION = 0.01; // [m]
    constexpr static double ERROR_HEADING = Math::deg2rad<double>(1.0); // [rad]
    constexpr static double ERROR_LINEAR_VELOCITY = 0.006; // [m/s]
    constexpr static double ERROR_ANGULAR_VELOCITY = 0.006; // [m/s]

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

#include <Robot.cpp>

#endif // ROBOT_HPP_
