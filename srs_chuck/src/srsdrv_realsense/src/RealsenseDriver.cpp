#include <RealsenseDriver.h>

#include <ros/ros.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
RealsenseDriver::RealsenseDriver() :
	rosNodeHandle_(),
	infrared1Subscriber_( rosNodeHandle_.subscribe("/camera/infrared1/image_raw", 10, &RealsenseDriver::onInfrared1, this) ),
	infrared2Subscriber_( rosNodeHandle_.subscribe("/camera/infrared2/image_raw", 10, &RealsenseDriver::onInfrared2, this) ),
	pointCloudSubscriber_( rosNodeHandle_.subscribe("/camera/depth/points", 10, &RealsenseDriver::onPointCloud, this) )
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void RealsenseDriver::run() {
	ros::Rate refreshRate(REFRESH_RATE_HZ);

	while (ros::ok()) {
		ros::spinOnce();

		refreshRate.sleep();
	}
}

void RealsenseDriver::onInfrared1( const sensor_msgs::Image& infraredImage1 )
{
	ROS_DEBUG_NAMED( "RealsenseDriver", "Infrared 1 image" );
}

void RealsenseDriver::onInfrared2( const sensor_msgs::Image& infraredImage2 )
{
	ROS_DEBUG_NAMED( "RealsenseDriver", "Infrared 2 image" );
}

void RealsenseDriver::onPointCloud( const sensor_msgs::PointCloud2& pointCloud )
{
	ROS_DEBUG_NAMED( "RealsenseDriver", "Point cloud received" );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

}// namespace srs
