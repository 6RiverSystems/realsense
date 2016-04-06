/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <filter/ukf/UnscentedKalmanFilter.hpp>

// Declare a test
TEST(TestSuite, CreateFilter)
{
    srs::UnscentedKalmanFilter<CV_64F> ukf(5, 1.0, 0.0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
