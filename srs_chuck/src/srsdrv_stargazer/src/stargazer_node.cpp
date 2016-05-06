/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <ros/ros.h>
#include <StarGazer.h>

const std::string g_SerialPort( "/dev/ftdi" );

const std::string g_strApsTopic( "/sensors/aps/raw" );

int main( int argc, char** argv )
{
	// Initialize ROS
	ros::init( argc, argv, "srsdrv_stargazer" );

	ROS_INFO_STREAM( "srsdrv_stargazer started" );

	// Create the StarGazer node and run it
	srs::StarGazer StarGazer( g_SerialPort, g_strApsTopic );

	StarGazer.run( );

	return 0;
}
