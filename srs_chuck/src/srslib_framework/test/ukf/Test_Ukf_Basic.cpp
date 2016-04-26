/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/filter/ukf/UnscentedKalmanFilter.hpp>

constexpr unsigned int UKF_STATE_SIZE = 5;

constexpr double ALPHA = 1.0;
constexpr double BETA = 0.0;

TEST(Test_Ukf, FilterCreation)
{
    // Create standard robot process model
    //Robot<> robot;

    //UnscentedKalmanFilter<UKF_STATE_SIZE> ukf(ALPHA, BETA, robot);
}
