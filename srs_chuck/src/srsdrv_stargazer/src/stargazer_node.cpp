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
    const static std::string NODE_NAME = "srsdrv_stargazer";

	// Initialize ROS
	ros::init( argc, argv, NODE_NAME );

	ROS_INFO_STREAM( NODE_NAME << " started" );

	// Create the StarGazer node and run it
	srs::StarGazer StarGazer( NODE_NAME, g_SerialPort, g_strApsTopic );

	StarGazer.Run( );

	return 0;
}
