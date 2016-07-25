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

	cv::Mat combinedIRImage;
	CombineImages( cvImage1->image, cvImage2->image, combinedIRImage );

	ThresholdImage( cvImage1->image );

	infrared1Publisher_.publish( cvImage1->toImageMsg( ) );

	ThresholdImage( cvImage2->image );

	infrared2Publisher_.publish( cvImage2->toImageMsg( ) );

	ThresholdImage( combinedIRImage );

	cv_bridge::CvImage combinedMsg;
	combinedMsg.header   = infraredImage2->header;
	combinedMsg.encoding = infraredImage2->encoding;
	combinedMsg.image    = combinedIRImage;

	infraredPublisher_.publish( combinedMsg );

	const uint32_t numberOfScans = scan->ranges.size( );

	const uint32_t xDiff = combinedIRImage.cols - numberOfScans;

	// TODO:  We currently have a problem if the image is than the number of scans
	const uint32_t xStart = xDiff/2;

	const double irRange = 0.3f;
	const double distanceFromBot = 0.01f;
	const double cameraFOV = 70.0f;
	// Calculate the maximum FOV width based on the ir range
	const double cameraMaxYOffset = irRange * tan( cameraFOV / 2.0f * M_PI / 180.0f );
	const double irMaxHeight = 0.5f; // 50%

	sensor_msgs::LaserScan irScan;
	irScan.header			= scan->header;
	irScan.angle_min		= scan->angle_min;
	irScan.angle_max		= scan->angle_max;
	irScan.angle_increment	= scan->angle_increment;
	irScan.time_increment	= scan->time_increment;
	irScan.scan_time		= scan->scan_time;
	irScan.range_min		= 0.0f;
	// Max distance should be the hypotenuse of the triangle
	irScan.range_max		= sqrt((cameraMaxYOffset*cameraMaxYOffset)+(distanceFromBot*distanceFromBot));
	irScan.ranges.assign( numberOfScans, std::numeric_limits<double>::infinity( ) );

	int maxHeight = combinedIRImage.rows / 2;

	for( int x = 0; x < numberOfScans; x++ )
	{
		double angle = ((double)x * irScan.angle_increment) + irScan.angle_min;

		for( int y = 0; y < maxHeight; y++ )
		{
			if( combinedIRImage.at<uint8_t>(y, x + xStart) == 128 )
			{
				//            y Offset
				//             ------
				//             \    |
				//              \   |
				// scan distance \  | Distance from bot
				//                \0|
				//                 \|

				double scanDistance = distanceFromBot / cos( angle );
				double yOffset = abs( distanceFromBot * sin( angle ) );

				// Only include data within the FOV of the camera
				if( yOffset <= cameraMaxYOffset )
				{
					if( irScan.ranges[x] == std::numeric_limits<double>::infinity( ) )
					{
						irScan.ranges[x] = scanDistance;
					}
					else
					{
						irScan.ranges[x] = scanDistance > irScan.ranges[x] ? scanDistance : irScan.ranges[x];
					}
				}
			}
		}
	}

	infraredScanPublisher_.publish( irScan );

	sensor_msgs::LaserScan irCombined;
	irCombined.header			= scan->header;
	irCombined.angle_min		= scan->angle_min;
	irCombined.angle_max		= scan->angle_max;
	irCombined.angle_increment	= scan->angle_increment;
	irCombined.time_increment	= scan->time_increment;
	irCombined.scan_time		= scan->scan_time;
	irCombined.range_min		= std::min( irScan.range_min, scan->range_min );
	irCombined.range_max		= std::max( irScan.range_max, scan->range_max );
	irCombined.ranges.assign( numberOfScans, std::numeric_limits<double>::infinity( ) );

	for( int x = 0; x < numberOfScans; x++ )
	{
		if( irScan.ranges[x] == std::numeric_limits<double>::infinity( ) )
		{
			// Use the depth scan
			irCombined.ranges[x] = scan->ranges[x];
		}
		else if( scan->ranges[x] == std::numeric_limits<double>::infinity( ) )
		{
			// Use the ir scan
			irCombined.ranges[x] = irScan.ranges[x];
		}
		else
		{
			// Use the closests range
			irCombined.ranges[x] = std::min( scan->ranges[x], irScan.ranges[x] );
		}
	}

	combinedScanPublisher_.publish( irCombined );
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
