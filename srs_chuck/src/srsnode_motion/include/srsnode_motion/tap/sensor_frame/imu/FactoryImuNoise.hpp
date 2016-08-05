/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef FACTORYIMUNOISE_HPP_
#define FACTORYIMUNOISE_HPP_

#include <cmath>

#include <opencv2/opencv.hpp>

#include <srsnode_motion/Configuration.hpp>

#include <srsnode_motion/MotionConfig.h>
using namespace srsnode_motion;

template<typename TYPE = double>
struct FactoryImuNoise
{
    static cv::Mat fromConfiguration(MotionConfig& configuration)
    {
        cv::Mat R = (cv::Mat_<TYPE>(1, STATIC_UKF_STATE_VECTOR_SIZE) <<
            0.0, // [m^2]
            0.0, // [m^2]
            pow(configuration.ukf_imu_error_yaw, 2.0), // [rad^2]
            0.0, // [m^2/s^2]
            0.0  // [rad^2/s^2]
        );

        return cv::Mat::diag(R);
    }
};

#endif // FACTORYIMUNOISE_HPP_
