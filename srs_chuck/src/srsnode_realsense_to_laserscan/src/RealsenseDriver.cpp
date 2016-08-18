#include <RealsenseDriver.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>

namespace srs
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods
////////////////////////////////////////////////////////////////////////////////////////////////////
RealsenseDriver::RealsenseDriver( ) :
	rosNodeHandle_( ),
	depthSubscriber_( rosNodeHandle_.subscribe<sensor_msgs::Image>( "/camera/depth/image_raw", 100,
		std::bind( &RealsenseDriver::OnDepthData, this, std::placeholders::_1 ) ) ),
	depthMedianFilterPublisher_( rosNodeHandle_.advertise<sensor_msgs::Image>( "/camera/depth/image_median_filter", 10 ) )
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void RealsenseDriver::run( )
{
	ros::Rate refreshRate( REFRESH_RATE_HZ );

	while( ros::ok( ) )
	{
		ros::spinOnce( );

		refreshRate.sleep( );
	}
}

void RealsenseDriver::OnDepthData( const sensor_msgs::Image::ConstPtr& depthImage )
{
	cv_bridge::CvImagePtr cvDepthImage = GetCvImage( depthImage );

	cv::flip(cvDepthImage->image, cvDepthImage->image, -1);

	cv::Mat medianFilterImg;
	cv::medianBlur( cvDepthImage->image, medianFilterImg, 5 );

	cvDepthImage->image = medianFilterImg;
	depthMedianFilterPublisher_.publish( cvDepthImage );
}

cv_bridge::CvImagePtr RealsenseDriver::GetCvImage( const sensor_msgs::Image::ConstPtr& image ) const
{
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr = cv_bridge::toCvCopy( image, image->encoding );
	}
	catch( cv_bridge::Exception& e )
	{
		ROS_ERROR( "cv_bridge exception: %s", e.what( ) );
	}

	return cv_ptr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

}// namespace srs
