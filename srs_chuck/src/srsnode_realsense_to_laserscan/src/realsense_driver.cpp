#include <ros/ros.h>

#include <RealsenseDriver.h>

int main( int argc, char** argv )
{
        ROS_ERROR("never been here");
	// Initialize ROS
	ros::init( argc, argv, "srsdrv_realsense" );

	ROS_INFO_STREAM( "srsdrv_realsense started" );

	srs::RealsenseDriver realsenseDriver;

	realsenseDriver.run( );

	return 0;
}
