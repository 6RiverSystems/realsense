#include <RealsenseDriver.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/contrib/contrib.hpp>
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
	depthMedianFilterPublisher_( rosNodeHandle_.advertise<sensor_msgs::Image>( "/camera/depth/image_median_filter", 10 ) ),
	depthBilateralFilterPublisher_( rosNodeHandle_.advertise<sensor_msgs::Image>( "/camera/depth/image_bilateral_filter", 10 ) ),
	depthColorPublisher_( rosNodeHandle_.advertise<sensor_msgs::Image>( "/camera/depth/image_color", 10 ) )
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

cv::Mat applyCustomColorMap(cv::Mat& im_gray)
{
	// Create look-up-table:
	cv::Mat lookUpTable(1, 256, CV_8UC3);

	for( int i = 0; i < 256; ++i)
	{
		cv::Vec3b color;


		if( i > 0 && i <= 96 )
		{
			double scale = 1.0f - ((double)96 - (double)i)/(double)96*0.5f;
			color = cv::Vec3b(0,0,255*scale); // 0.0m => 0.75m;
		}
		else if( i <= 128 )
		{
			double scale = 1.0f - ((double)128 - (double)i)/(double)128*0.5f;
			color = cv::Vec3b(0,255*scale,0); // 0.75m => 2.0m;
		}
		else
		{
			color = cv::Vec3b(255,102,102);	 // 2.0m => 8.0m
		}

		lookUpTable.at<cv::Vec3b>(0,i) = color;
	}

	cv::Mat input8UC3;
	cv::cvtColor( im_gray, input8UC3, CV_GRAY2BGR );

	cv::Mat output;
	cv::LUT(input8UC3, lookUpTable, output);

	return output;
}

void RealsenseDriver::OnDepthData( const sensor_msgs::Image::ConstPtr& depthImage )
{
	cv_bridge::CvImagePtr cvDepthImage = GetCvImage( depthImage );

	cv::flip(cvDepthImage->image, cvDepthImage->image, -1);

	cv::Mat medianFilterImg;
	cv::medianBlur( cvDepthImage->image, medianFilterImg, 5 );

	cvDepthImage->image = medianFilterImg;
	depthMedianFilterPublisher_.publish( cvDepthImage );

	double maxDistance = 2000.0f;
	cv::Mat adjMap;
	cv::convertScaleAbs(medianFilterImg, adjMap, 255 / maxDistance);

	// The number of bins
	int histSize = 256;

	// Set the ranges
	float range[] = { 0, 256 } ;
	const float* histRange = { range };

	bool uniform = true; bool accumulate = false;

	cv::Mat histMap;
	cv::calcHist( &adjMap, 1, 0, cv::Mat(), histMap, 1, &histSize, &histRange, uniform, accumulate );

	// Draw the histogram
	int hist_w = 512; int hist_h = 400;
	int bin_w = cvRound( (double) hist_w/histSize );

	cv::Mat histImage( hist_h, hist_w, CV_8UC1, cv::Scalar(0,0,0) );
	cv::normalize(histMap, histMap, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

	// Draw for each channel
	for( int i = 1; i < histSize; i++ )
	{
		cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(histMap.at<float>(i-1)) ) ,
			cv::Point( bin_w*(i), hist_h - cvRound(histMap.at<float>(i)) ),
			cv::Scalar( 255), 2, 8, 0  );
	}

	cv::Mat colorMap = applyCustomColorMap( adjMap );

	depthColorPublisher_.publish( cv_bridge::CvImage(depthImage->header, "bgr8", colorMap).toImageMsg() );

	depthBilateralFilterPublisher_.publish( cv_bridge::CvImage(depthImage->header, "mono8", histImage).toImageMsg() );
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
