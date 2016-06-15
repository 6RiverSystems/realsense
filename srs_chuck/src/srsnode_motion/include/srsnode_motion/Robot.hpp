/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <opencv2/opencv.hpp>

#include <srslib_framework/filter/Process.hpp>
#include <srslib_framework/filter/Command.hpp>
#include <srslib_framework/math/AngleMath.hpp>

#include <srsnode_motion/Configuration.hpp>
#include <srsnode_motion/StatePe.hpp>

namespace srs {

template<int TYPE = CV_64F>
class Robot : public Process<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>
{
public:
    typedef typename Process<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>::BaseType BaseType;

    // Standard deviation values for the robot process
    constexpr static double ERROR_LOCATION = 0.001; // [m]
    constexpr static double ERROR_HEADING = AngleMath::deg2rad<double>(0.1); // [rad]
    constexpr static double ERROR_LINEAR_VELOCITY = 0.006; // [m/s]
    constexpr static double ERROR_ANGULAR_VELOCITY = 0.006; // [m/s]

    constexpr static double ANGULAR_VELOCITY_EPSILON = 0.001; // [rad/s] (0.0573 [deg/s])

    Robot() :
        Process<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>(cv::Mat::diag(Q))
    {}

    virtual ~Robot()
    {}

    virtual cv::Mat FB(
        const cv::Mat stateVector,
        Command<STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>* const command,
        BaseType dT);

    void kinematics(StatePe<TYPE> sT0, BaseType dT, StatePe<TYPE>& sT1);

private:
    const static cv::Mat Q;
};

} // namespace srs

#include <Robot.cpp>

#endif // ROBOT_HPP_
