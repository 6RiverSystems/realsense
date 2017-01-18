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
        depthPublisher_( rosNodeHandle_.advertise<sensor_msgs::Image>("/interal/sensors/rgbd/depth/image_filtered", 100 ) )
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
        cv_bridge::CvImagePtr cvColorImage = GetCvImage( colorImage );
        cv::imshow("color", cvColorImage->image);
        //cv::imwrite("color.jpg", cvColorImage->image);
        cv::waitKey(10);
}
void RealsenseDriver::OnDepthData( const sensor_msgs::Image::ConstPtr& depthImage )
{
	cv_bridge::CvImagePtr cvDepthImage = GetCvImage( depthImage );

	cv::Mat tmpImg;
	tmpImg = cvDepthImage->image.clone();
	cv::Size size(480, 360);
	//cv::flip(cvDepthImage->image, cvDepthImage->image, -1);

	// normalize 16-bit unsigned range to 8-bit for visualization
	double max, min;
	cv::minMaxIdx(tmpImg, &min, &max);
	ROS_DEBUG("min: %f, max: %f", min, max);
    	cv::convertScaleAbs(tmpImg, tmpImg, 255/max);
    	tmpImg.convertTo(tmpImg, CV_8UC1);
    	cv::resize(tmpImg, tmpImg, size, 0, 0, cv::INTER_AREA);
    	cv::imshow("raw image", tmpImg);
    	//cv::imwrite("depth.jpg", tmpImg);
    	// apply median filter
    	cv::Mat medianFilterImg;
    	cv::medianBlur( tmpImg, medianFilterImg, 5 );
    	cv::imshow("median filtered image", medianFilterImg);

    	// apply bilateral filter
    	//cv::Mat bilateralImg;
    	//cv::bilateralFilter ( tmpImg, bilateralImg, 15, 80, 80 );
    	//cv::imshow("bilateral image", bilateralImg);
    
    	// apply joint bilateral filter
    	//cv::Mat jbilateralImg;
    	//cv::ximgproc::jointBilateralFilter(tmpImg, tmpImg, jbilateralImg, 15, 80, 80);
    	//depthMedianFilterPublisher_.publish( cvDepthImage );
    	cv::waitKey(10);
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
