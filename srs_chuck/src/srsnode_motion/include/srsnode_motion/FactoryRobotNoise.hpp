/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef FACTORYROBOTNOISE_HPP_
#define FACTORYROBOTNOISE_HPP_

#include <cmath>

#include <opencv2/opencv.hpp>

#include <srsnode_motion/Configuration.hpp>

#include <srsnode_motion/MotionConfig.h>
using namespace srsnode_motion;

// TODO: At the moment the odometry noise is included in the robot model
// because odometry is treated like a command. Moving odometry to be a sensor
// requires those values to be 0
template<typename TYPE = double>
struct FactoryRobotNoise
{
    static cv::Mat fromConfiguration(MotionConfig& configuration)
    {
        cv::Mat Q = (cv::Mat_<TYPE>(1, STATIC_UKF_STATE_VECTOR_SIZE) <<
            pow(configuration.ukf_robot_error_location, 2.0), // [m^2]
            pow(configuration.ukf_robot_error_location, 2.0), // [m^2]
            pow(configuration.ukf_robot_error_heading, 2.0), // [rad^2]
            pow(configuration.ukf_odometry_error_linear, 2.0), // [m^2/s^2]
            pow(configuration.ukf_imu_error_yaw_rot, 2.0) // [m^2/s^2]
        );

        return cv::Mat::diag(Q);
    }
};

#endif // FACTORYROBOTNOISE_HPP_
