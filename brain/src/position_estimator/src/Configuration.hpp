/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef CONFIGURATION_HPP_
#define CONFIGURATION_HPP_

#include <opencv2/opencv.hpp>

// Preprocessor-level definition of the size of the state vector
// in the estimator
#define STATIC_UKF_STATE_VECTOR_SIZE 5
#define STATIC_UKF_COMMAND_VECTOR_SIZE 2

#define STATIC_UKF_CV_TYPE CV_64F

#endif // CONFIGURATION_HPP_
