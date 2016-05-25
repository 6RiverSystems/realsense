#include <RealsenseDriver.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>

namespace srs
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods
////////////////////////////////////////////////////////////////////////////////////////////////////
RealsenseDriver::RealsenseDriver( ) :
	rosNodeHandle_( ),
	infrared1Subscriber_( rosNodeHandle_, "/camera/infrared1/image_raw", 10 ),
	infrared2Subscriber_( rosNodeHandle_, "/camera/infrared2/image_raw", 10 ),
	laserScanSubscriber_( rosNodeHandle_, "/camera/depth/scan", 10 ),
	infrared1Publisher_( rosNodeHandle_.advertise<sensor_msgs::Image>( "/camera/infrared1/image_near", 10 ) ),
	infrared2Publisher_( rosNodeHandle_.advertise<sensor_msgs::Image>( "/camera/infrared2/image_near", 10 ) ),
	infraredPublisher_( rosNodeHandle_.advertise<sensor_msgs::Image>( "/camera/infrared/image_near", 10 ) ),
	infraredScanPublisher_( rosNodeHandle_.advertise<sensor_msgs::LaserScan>( "/camera/infrared/scan", 10 ) ),
	combinedScanPublisher_( rosNodeHandle_.advertise<sensor_msgs::LaserScan>( "/camera/scan", 10 ) ),
	synchronizer_( new ImageSyncronizer( laserScanSubscriber_, infrared1Subscriber_, infrared2Subscriber_, 10 ) )
{
	synchronizer_->registerCallback(
		std::bind( &RealsenseDriver::OnDepthData, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3 ) );
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

void RealsenseDriver::OnDepthData( const sensor_msgs::LaserScan::ConstPtr& scan, const sensor_msgs::Image::ConstPtr& infraredImage1,
	const sensor_msgs::Image::ConstPtr& infraredImage2 )
{
	cv_bridge::CvImagePtr cvImage1 = GetCvImage( infraredImage1 );

	cv_bridge::CvImagePtr cvImage2 = GetCvImage( infraredImage2 );

	cv::Mat combinedImages;
	CombineImages( cvImage1->image, cvImage2->image, combinedImages );

	ThresholdImage( cvImage1->image );

	infrared1Publisher_.publish( cvImage1->toImageMsg( ) );

	ThresholdImage( cvImage2->image );

	infrared2Publisher_.publish( cvImage2->toImageMsg( ) );

	ThresholdImage( combinedImages );

	cv_bridge::CvImage combinedMsg;
	combinedMsg.header   = infraredImage2->header;
	combinedMsg.encoding = infraredImage2->encoding;
	combinedMsg.image    = combinedImages;

	infraredPublisher_.publish( combinedMsg );

	sensor_msgs::LaserScan irScan;
	irScan.header			= scan->header;
	irScan.angle_min		= scan->angle_min;
	irScan.angle_max		= scan->angle_max;
	irScan.angle_increment	= scan->angle_increment;
	irScan.time_increment	= scan->time_increment;
	irScan.scan_time		= scan->scan_time;
	irScan.range_min		= 0.0f;
	irScan.range_max		= 1.0f;

	const uint32_t numberOfScans = scan->ranges.size( );

	irScan.ranges.assign( numberOfScans, std::numeric_limits<double>::infinity( ) );

	const uint32_t xDiff = combinedImages.cols - numberOfScans;

	// TODO:  We currently have a problem if the image is than the number of scans
	const uint32_t xStart = xDiff/2;

	const double distanceFromBot = 0.300;
	const double robotHalfWidth = 0.310 / 2.0f;

	for( int x = 0; x < numberOfScans; x++ )
	{
		double angle = (x * irScan.angle_increment) + irScan.angle_min;

		for( int y = 0; y < combinedImages.rows; y++ )
		{
			if( combinedImages.at<uint8_t>(y, x + xStart) == 128 )
			{
				double xDistance = distanceFromBot / cos( angle );
				double yDistance = abs( distanceFromBot * sin( angle ) );

				// Only include data within the footprint (width) of the robot
				if( yDistance <= robotHalfWidth )
				{
					if( irScan.ranges[x] == std::numeric_limits<double>::infinity( ) )
					{
						irScan.ranges[x] = xDistance;
					}
					else
					{
						irScan.ranges[x] = xDistance > irScan.ranges[x] ? xDistance : irScan.ranges[x];
					}
				}
			}
		}
	}

	infraredScanPublisher_.publish( irScan );

	for( int x = 0; x < numberOfScans; x++ )
	{
		if( irScan.ranges[x] == std::numeric_limits<double>::infinity( ) )
		{
			// Use the depth scan
			irScan.ranges[x] = scan->ranges[x];
		}
		else if( scan->ranges[x] != std::numeric_limits<double>::infinity( ) )
		{
			// If the depth scan is valid then pick the closest scan
			irScan.ranges[x] = std::min( scan->ranges[x], scan->ranges[x] );
		}
	}

	combinedScanPublisher_.publish( irScan );
}

cv_bridge::CvImagePtr RealsenseDriver::GetCvImage( const sensor_msgs::Image::ConstPtr& image ) const
{
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr = cv_bridge::toCvCopy( image, sensor_msgs::image_encodings::TYPE_8UC1 );
	}
	catch( cv_bridge::Exception& e )
	{
		ROS_ERROR( "cv_bridge exception: %s", e.what( ) );
	}

	return cv_ptr;
}

void RealsenseDriver::ThresholdImage( cv::Mat& image ) const
{
	// Apply a binary threshold filter to remove all but the saturated pixels
	cv::threshold( image, image, 254, 128, cv::THRESH_BINARY );
}

void RealsenseDriver::CombineImages( cv::Mat& image1, cv::Mat& image2, cv::Mat& result ) const
{
	if( image1.cols == image2.cols &&
		image1.rows == image2.rows )
	{
		constexpr uint32_t irOffset = 70;

		// Further optimization: Use stitching algorithm to align images (not sure it's worth the cycles though)
		cv::Mat combined( image1.rows, image1.cols + irOffset, image1.type( ) );

		image1.copyTo( combined( cv::Rect( 0, 0, image1.cols, image1.rows ) ) );
		image2.copyTo( combined( cv::Rect( irOffset, 0, image2.cols, image2.rows ) ) );

		result = combined;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

}// namespace srs
