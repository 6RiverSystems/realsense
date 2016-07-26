#include <RealsenseDriver.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/highgui/highgui.hpp>

namespace srs
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods
////////////////////////////////////////////////////////////////////////////////////////////////////
RealsenseDriver::RealsenseDriver( ) :
	rosNodeHandle_( ),
	infrared1Subscriber_( rosNodeHandle_, "/camera/infrared1/image_raw", 10 ),
	infrared2Subscriber_( rosNodeHandle_, "/camera/infrared2/image_raw", 10 ),
	infraredPublisher_( rosNodeHandle_.advertise<sensor_msgs::Image>( "/camera/infrared/image_near", 10 ) ),
	infraredScanPublisher_( rosNodeHandle_.advertise<sensor_msgs::LaserScan>( "/camera/infrared/scan", 10 ) ),
	synchronizer_( new ImageSyncronizer( infrared1Subscriber_, infrared2Subscriber_, 10 ) )
{
	synchronizer_->registerCallback( std::bind(&RealsenseDriver::OnInfraredData, this,
		std::placeholders::_1, std::placeholders::_2));
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

void RealsenseDriver::OnInfraredData( const sensor_msgs::Image::ConstPtr& infraredImage1,
	const sensor_msgs::Image::ConstPtr& infraredImage2 )
{
	cv_bridge::CvImagePtr cvImage1 = GetCvImage( infraredImage1 );

	cv_bridge::CvImagePtr cvImage2 = GetCvImage( infraredImage2 );

	ThresholdImage( cvImage1->image );

	ThresholdImage( cvImage2->image );

	cv::Mat combinedIRImage;
	//CombineImages( cvImage1->image, cvImage2->image, combinedIRImage );

	combinedIRImage = cvImage1->image -  cvImage2->image;
	//	cv::medianBlur( combinedIRImage, combinedIRImage, 7 );

	cv_bridge::CvImage combinedMsg;
	combinedMsg.header   = infraredImage2->header;
	combinedMsg.encoding = infraredImage2->encoding;
	combinedMsg.image    = combinedIRImage;

	const uint32_t numberOfScans = combinedIRImage.cols;

	const double realsenseHorizontalFOV = 70.0f;
	const double realsenseVerticalFOV = 43.0f;

	const double irRange = 0.3f;
	const double distanceFromBot = 0.01f;
	const double cameraFOV = 70.0f;
	// Calculate the maximum FOV width based on the ir range
	const double cameraMaxYOffset = irRange * tan( cameraFOV / 2.0f * M_PI / 180.0f );
	const double irMaxHeight = 0.5f; // 50%

	sensor_msgs::LaserScan irScan;
	irScan.header.frame_id = "laser_frame";
	irScan.angle_min		= -(realsenseHorizontalFOV / 2.0f);
	irScan.angle_max		=  (realsenseHorizontalFOV / 2.0f);
	irScan.angle_increment	= (irScan.angle_max - irScan.angle_min) / (double)numberOfScans;
	irScan.time_increment	= 0.033;
	irScan.scan_time		= 0.033;
	irScan.range_min		= 0.0f;
	// Max distance should be the hypotenuse of the triangle
	irScan.range_max		= sqrt((cameraMaxYOffset*cameraMaxYOffset)+(distanceFromBot*distanceFromBot));
	irScan.ranges.assign( numberOfScans, std::numeric_limits<double>::infinity( ) );

	int maxHeight = combinedIRImage.rows / 2;

	double irMinValue = std::numeric_limits<double>::infinity( );
	double depthMinValue = std::numeric_limits<double>::infinity( );
	uint32_t irCount = 0;
	uint32_t depthCount = 0;
	uint32_t combinedCount = 0;

	for( int x = 0; x < numberOfScans; x++ )
	{
		double angle = ((double)x * irScan.angle_increment) + irScan.angle_min;

		for( int y = 0; y < maxHeight; y++ )
		{
			if( combinedIRImage.at<uint8_t>(y, x) == THRESHOLD_VALUE )
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

						irMinValue = std::min<double>( irMinValue, distanceFromBot );
						irCount++;
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

	infraredPublisher_.publish( combinedMsg );
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
	cv::threshold( image, image, 80, THRESHOLD_VALUE, cv::THRESH_BINARY );
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
