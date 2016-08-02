/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef FACTORYAPSNOISE_HPP_
#define FACTORYAPSNOISE_HPP_

#include <cmath>

#include <opencv2/opencv.hpp>

#include <srsnode_motion/Configuration.hpp>

#include <srsnode_motion/MotionConfig.h>
using namespace srsnode_motion;

struct FactoryApsNoise
{
    static cv::Mat fromConfiguration(MotionConfig& configuration)
    {
        // TODO Fix this so that double comes from a template
        cv::Mat R = (cv::Mat_<double>(1, STATIC_UKF_STATE_VECTOR_SIZE) <<
            pow(configuration.ukf_aps_error_location, 2.0), // [m^2]
            pow(configuration.ukf_aps_error_location, 2.0), // [m^2]
            pow(configuration.ukf_aps_error_heading, 2.0), // [rad^2]
            0.0, // [m^2/s^2]
            0.0 // [m^2/s^2]
        );

        return cv::Mat::diag(R);
    }
};

#endif // FACTORYAPSNOISE_HPP_
