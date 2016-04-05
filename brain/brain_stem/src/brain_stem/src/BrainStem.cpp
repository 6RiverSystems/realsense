#include <ros/ros.h>
#include "SerialIO.h"

////////////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////////////

using namespace srs;

void readCallback( std::vector<char> data )
{
	ROS_INFO("Serial Data Recieved: %d", (int)data.size( ) );

}

int main(int argc, char **argv)
{
	ROS_INFO("Starting brain stem");

	SerialIO serialIO;

	serialIO.Open( "/dev/tnt3", readCallback );

	// Initialize ROS stuff
	ros::init( argc, argv, "brain_stem" );
	ros::NodeHandle node;

	// Respond to inputs until shut down
	ros::spin();

	return 0;
}
