/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
	ros::Time::init();

	testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}
