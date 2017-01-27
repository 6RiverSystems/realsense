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
	depthSubscriber_( rosNodeHandle_.subscribe<sensor_msgs::Image>( "/internal/sensors/rgbd/depth/image_raw", 100,
		std::bind( &RealsenseDriver::OnDepthData, this, std::placeholders::_1 ) ) ),
        depthPublisher_( rosNodeHandle_.advertise<sensor_msgs::Image>("/internal/sensors/rgbd/depth/image_filtered", 100 ) )
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

	cv::Mat cloneImage;
	cloneImage = cvDepthImage->image.clone();
	//cv::flip(cvDepthImage->image, cvDepthImage->image, -1);

	// since many operations require image in CV_8UC1 format, and thus we normalize 16-bit to 8-bit
	double max, min;
	cv::minMaxIdx(cloneImage, &min, &max);
	double ratio = 255 / max;
	cloneImage.convertTo(cloneImage, CV_8UC1, ratio);

	// first operation- thresholding image with any value closer than 0.3 (meter) or 3 (meter) further
	cv::threshold(cloneImage, cloneImage, 300 * ratio, 255, cv::THRESH_TOZERO);
	cv::threshold(cloneImage, cloneImage, 3000 * ratio, 255, cv::THRESH_TRUNC);

	// second step- apply median filter, kernel size 3 and 5 are good options
	cv::Mat medianFilterImage;
	cv::medianBlur( cloneImage, medianFilterImage, 3 );

        // third step- scale image back to CV_16UC1
	medianFilterImage.convertTo(medianFilterImage, CV_16UC1, 1 / ratio);

	cvDepthImage->image = medianFilterImage;
	depthPublisher_.publish( cvDepthImage );
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
