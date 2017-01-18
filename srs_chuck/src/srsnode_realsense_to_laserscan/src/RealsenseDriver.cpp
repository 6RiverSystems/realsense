#include <RealsenseDriver.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgproc/ximgproc.hpp>
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
        colorSubscriber_( rosNodeHandle_.subscribe<sensor_msgs::Image>( "/internal/sensors/rgbd/color/image_raw", 100, std::bind( &RealsenseDriver::OnColorData, this, std::placeholders::_1 ) ) ),
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
void RealsenseDriver::OnColorData( const sensor_msgs::Image::ConstPtr& colorImage ){
        //cv_bridge::CvImagePtr cvColorImage = GetCvImage( colorImage );
        //cv::imshow("color", cvColorImage->image);
        //cv::imwrite("color.jpg", cvColorImage->image);
        //cv::waitKey(10);
}
void RealsenseDriver::OnDepthData( const sensor_msgs::Image::ConstPtr& depthImage )
{
	cv_bridge::CvImagePtr cvDepthImage = GetCvImage( depthImage );

    cv::Mat cloneImage;
    cloneImage = cvDepthImage->image.clone();
    //cv::Size size(480, 360);
    //cv::flip(cvDepthImage->image, cvDepthImage->image, -1);

    // since many operations require 8-bit image, and thus we normalize 16-bit to 8-bit
    double max, min;
    cv::minMaxIdx(cloneImage, &min, &max);
    ROS_DEBUG("before min: %f, max: %f", min, max);
    double ratio = 255 / max;
    // normalize range and convert type
    //cv::resize(cloneImage, cloneImage, size, 0, 0);
    cloneImage.convertTo(cloneImage, CV_8UC1, ratio);
    //cv::imshow("raw image", cloneImage);

    // first operation- thresholding image with any value closer than 300 mm
    cv::threshold(cloneImage, cloneImage, 300 * ratio, 255, cv::THRESH_TOZERO);

    // second step- apply median filter
    cv::Mat medianFilterImage;
    cv::medianBlur( cloneImage, medianFilterImage, 5 );
    //cv::imshow("median filtered image", medianFilterImage);

    // third step- retrieve the value from medianFilterImage mask
    cv::Mat outputImage;
    cloneImage.copyTo(outputImage, medianFilterImage);
    //cv::imshow("final image", outputImage);

    outputImage.convertTo(outputImage, CV_16UC1, 1 / ratio);
    cv::minMaxIdx(outputImage, &min, &max);
    ROS_DEBUG("after min: %f, max: %f", min, max);

    // apply bilateral filter
    //cv::Mat bilateralImg;
    //cv::bilateralFilter ( tmpImg, bilateralImg, 15, 80, 80 );
    //cv::imshow("bilateral image", bilateralImg);

    cvDepthImage->image = outputImage;
    depthPublisher_.publish( cvDepthImage );
    //cv::waitKey(10);
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
