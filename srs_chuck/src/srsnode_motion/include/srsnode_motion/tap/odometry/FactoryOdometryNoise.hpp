/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef FACTORYODOMETRYNOISE_HPP_
#define FACTORYODOMETRYNOISE_HPP_

#include <cmath>

#include <opencv2/opencv.hpp>

#include <srsnode_motion/Configuration.hpp>

#include <srsnode_motion/MotionConfig.h>
using namespace srsnode_motion;

struct FactoryOdometryNoise
{
    static cv::Mat fromConfiguration(MotionConfig& configuration)
    {
        // TODO Fix this so that double comes from a template
        cv::Mat R = (cv::Mat_<double>(1, STATIC_UKF_STATE_VECTOR_SIZE) <<
            0.0, // [m^2]
            0.0, // [m^2]
            0.0, // [rad^2]
            pow(configuration.ukf_odometry_error_linear, 2.0), // [m^2/s^2]
            pow(configuration.ukf_odometry_error_angular, 2.0) // [m^2/s^2]
        );

        return cv::Mat::diag(R);
    }
};

#endif // FACTORYODOMETRYNOISE_HPP_
