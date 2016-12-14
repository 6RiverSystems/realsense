/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <ros/ros.h>

#include "../include/srsdrv_brainstem/BrainStem.hpp"

const std::string g_SerialPort( "/dev/malg" );

int main( int argc, char** argv )
{
	// Initialize ROS
	ros::init( argc, argv, "srsdrv_brainstem" );

	ROS_INFO_STREAM( "srsdrv_brainstem started" );

	// Create the BrainStem node and run it
	srs::BrainStem brainStem( g_SerialPort );

	brainStem.Run( );

	return 0;
}
